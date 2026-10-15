#include "ota_upgrade.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "cJSON.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "mbedtls/pk.h"
#include "mbedtls/sha256.h"
#include "dap_core_service.h"
#include "network_tcp_server.h"

extern const unsigned char key_start[] asm("_binary_release_public_pem_start");
extern const unsigned char key_end[] asm("_binary_release_public_pem_end");
static uint8_t buffer[4096]; /* HTTP server serializes handlers. */
static char manifest[2049];
static uint8_t signature[384];
static esp_timer_handle_t reboot_timer;
static void reboot(void *arg) { (void)arg; esp_restart(); }
static bool receive_exact(httpd_req_t *r, void *out, size_t n) {
    size_t done=0;
    while(done<n) {
        int got=httpd_req_recv(r,(char *)out+done,n-done);
        if(got<=0)return false;
        done+=(size_t)got;
    }
    return true;
}
static uint32_t le32(const uint8_t *p) {
    return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
}
static bool equal(cJSON *m,const char *key,const char *value) {
    cJSON *v=cJSON_GetObjectItemCaseSensitive(m,key);
    return cJSON_IsString(v)&&strcmp(v->valuestring,value)==0;
}
esp_err_t ota_upgrade_receive(httpd_req_t *r) {
    if(network_tcp_server_has_client()||!dap_core_service_network_begin()) {
        httpd_resp_set_status(r,"409 Conflict");
        return httpd_resp_sendstr(r,"{\"error\":\"debug_or_network_busy\"}");
    }
    esp_ota_handle_t handle=0;
    bool begun=false,ended=false;
    cJSON *m=NULL;
    esp_err_t error=ESP_FAIL;
    const char *reason="invalid_package";
    mbedtls_pk_context pk;mbedtls_pk_init(&pk);
    mbedtls_sha256_context hash;mbedtls_sha256_init(&hash);
    uint8_t header[12],digest[32];
    char hex[65];
    if(!receive_exact(r,header,sizeof(header))||memcmp(header,"RLY1",4))goto fail;
    uint32_t ml=le32(header+4),sl=le32(header+8);
    if(!ml||ml>2048||sl!=sizeof(signature)||r->content_len>0x201000)goto fail;
    if(!receive_exact(r,manifest,ml)||!receive_exact(r,signature,sl))goto fail;
    manifest[ml]=0;
    if(memchr(manifest,0,ml))goto fail;
    if(mbedtls_sha256((uint8_t *)manifest,ml,digest,0)!=0)goto fail;
    reason="invalid_signature";
    if(mbedtls_pk_parse_public_key(&pk,key_start,key_end-key_start)!=0 ||
       mbedtls_pk_verify(&pk,MBEDTLS_MD_SHA256,digest,32,signature,sl)!=0)goto fail;
    m=cJSON_Parse(manifest);reason="incompatible_package";
    cJSON *size=m?cJSON_GetObjectItemCaseSensitive(m,"size"):NULL;
    cJSON *format=m?cJSON_GetObjectItemCaseSensitive(m,"format"):NULL;
    cJSON *version=m?cJSON_GetObjectItemCaseSensitive(m,"version"):NULL;
    if(!m||!cJSON_IsNumber(format)||format->valuedouble!=1||!cJSON_IsNumber(size)||
       size->valuedouble<288||size->valuedouble>0x200000||size->valuedouble!=(int)size->valuedouble||
       !cJSON_IsString(version)||strlen(version->valuestring)>31||
       !equal(m,"product","rlyoulink")||!equal(m,"hardware","xiaos3-8mb-nrst-d5")||
       !equal(m,"layout","amphilink-8mb-v1")||!equal(m,"target","inactive_ota_slot"))goto fail;
    size_t length=(size_t)size->valuedouble;
    if(r->content_len!=12+ml+sl+length)goto fail;
    const esp_partition_t *running=esp_ota_get_running_partition();
    const esp_partition_t *part=esp_ota_get_next_update_partition(NULL);
    if(!running||!part||part==running||part->size!=0x200000||
       (part->address!=0x20000&&part->address!=0x220000))goto fail;
    reason="write_failed";
    if(esp_ota_begin(part,length,&handle)!=ESP_OK)goto fail;
    begun=true;
    if(mbedtls_sha256_starts(&hash,0)!=0)goto fail;
    for(size_t done=0;done<length;) {
        size_t n=length-done;if(n>sizeof(buffer))n=sizeof(buffer);
        if(!receive_exact(r,buffer,n)||mbedtls_sha256_update(&hash,buffer,n)!=0||
           esp_ota_write(handle,buffer,n)!=ESP_OK)goto fail;
        done+=n;
    }
    if(mbedtls_sha256_finish(&hash,digest)!=0)goto fail;
    const char digits[]="0123456789abcdef";
    for(int i=0;i<32;i++){hex[i*2]=digits[digest[i]>>4];hex[i*2+1]=digits[digest[i]&15];}hex[64]=0;
    reason="hash_mismatch";if(!equal(m,"sha256",hex))goto fail;
    error=esp_ota_end(handle);ended=true;
    reason="invalid_image";if(error!=ESP_OK)goto fail;
    esp_app_desc_t desc;
    if(esp_ota_get_partition_description(part,&desc)!=ESP_OK||strcmp(desc.project_name,"rlyoulink")||
       strcmp(desc.version,version->valuestring))goto fail;
    if(!reboot_timer) {
        esp_timer_create_args_t args={.callback=reboot,.name="ota-reboot"};
        if(esp_timer_create(&args,&reboot_timer)!=ESP_OK)goto fail;
    }
    reason="boot_selection_failed";
    if(esp_ota_set_boot_partition(part)!=ESP_OK)goto fail;
    cJSON_Delete(m);mbedtls_pk_free(&pk);mbedtls_sha256_free(&hash);
    httpd_resp_set_type(r,"application/json");
    error=httpd_resp_sendstr(r,"{\"accepted\":true,\"state\":\"rebooting\"}");
    if(esp_timer_start_once(reboot_timer,500000)!=ESP_OK)esp_restart();
    return error;
fail:
    if(begun&&!ended)esp_ota_abort(handle);
    cJSON_Delete(m);mbedtls_pk_free(&pk);mbedtls_sha256_free(&hash);
    dap_core_service_network_end(NULL);
    httpd_resp_set_status(r,"400 Bad Request");
    httpd_resp_set_type(r,"application/json");
    char response[100];snprintf(response,sizeof(response),"{\"error\":\"%s\"}",reason);
    return httpd_resp_sendstr(r,response);
}
