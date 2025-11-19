/*
 * SofiOS project V1.0.0
 * Copyright (C) 2023 Sofiatech Tunisia. All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @file bsp_ble.c
 * @brief ble driver 
 */

/** @addtogroup ble
  * @{
  */

/*-----------------------------------------------------------------------------------------------*/
/* Includes                                                                                      */
/*-----------------------------------------------------------------------------------------------*/
#include "bsp_ble.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_uuid.h" 
#include "host/ble_hs_id.h"
#include "nvs.h"
#include "nvs_flash.h"

/*-----------------------------------------------------------------------------------------------*/
/* Defines                                                                                       */
/*-----------------------------------------------------------------------------------------------*/

#define MAX_SERVICES 5
#define MAX_CHARS_PER_SERVICE 4
#define MAX_VALUE_LEN 20 
/*-----------------------------------------------------------------------------------------------*/
/* Private types                                                                                 */
/*-----------------------------------------------------------------------------------------------*/
/** @defgroup ble_private_types Private types
  * @{
  */

  typedef enum {
    BSP_BLE_ROLE_NONE = 0,
    BSP_BLE_ROLE_PERIPHERAL,
    BSP_BLE_ROLE_CENTRAL
} bsp_ble_role_t;
/**
  * @}
  */

/*-----------------------------------------------------------------------------------------------*/
/* Private variables                                                                             */
/*-----------------------------------------------------------------------------------------------*/
/** @defgroup ble_private_variables Private variables
  * @{
  */
static uint8_t ble_addr_type;
static const char *TAG = "BSP_BLE_SERVICE";
static volatile uint16_t conn_handle = 0xFFFF;
static nvs_handle_t my_nvs_handle = 0;

  /* --- Handles et valeurs des caractéristiques --- */
static uint16_t char_handles[MAX_SERVICES][MAX_CHARS_PER_SERVICE] = {0};
static uint8_t char_values[MAX_SERVICES][MAX_CHARS_PER_SERVICE][MAX_VALUE_LEN];
static uint16_t char_value_len[MAX_SERVICES][MAX_CHARS_PER_SERVICE];

/* --- Services et caractéristiques --- */
static struct ble_gatt_svc_def my_services[MAX_SERVICES + 1];
static ble_uuid16_t service_uuids[MAX_SERVICES];
static int service_count = 0;
static struct ble_gatt_chr_def my_characteristics[MAX_SERVICES][MAX_CHARS_PER_SERVICE + 1];
static ble_uuid16_t char_uuids[MAX_SERVICES][MAX_CHARS_PER_SERVICE];
static int char_count[MAX_SERVICES] = {0};

/* --- UUIDs utilisés pour READ/WRITE/NOTIFY/INDICATE --- */
static ble_uuid16_t g_uuids_read[4];
static ble_uuid16_t g_uuids_write[4];
static ble_uuid16_t g_uuids_notify[4];
static ble_uuid16_t g_uuids_indicate[4];
static int read_index = 0;
static int write_index = 0;
static int notify_index = 0;
static int indicate_index = 0;


/* --- Paramètres GAP Advertising --- */
static struct ble_gap_adv_params g_adv_params;
static char g_device_name[32] = {0};
static uint8_t g_conn_mode = 0;
static uint8_t g_disc_mode = 0;

/* --- Paramètres GAP Scanning --- */
static struct ble_gap_disc_params current_disc_params = {
    .itvl = 0x0010,
    .window = 0x0010,
    .filter_policy = 0,
    .limited = 0,
    .passive = 0,
    .filter_duplicates = 1
};

/* --- Paramètres GAP Connection --- */
static struct ble_gap_conn_params conn_params = {
    .scan_itvl = 0x0010,
    .scan_window = 0x0010,
    .itvl_min = 24,
    .itvl_max = 40,
    .latency = 0,
    .supervision_timeout = 400,
    .min_ce_len = 0,
    .max_ce_len = 0,
};
static bsp_ble_role_t current_role = BSP_BLE_ROLE_NONE;

static const uint8_t target_addr[6] = {0x52, 0x5D, 0xD9, 0x2F, 0x2B, 0x14}; // adresse esp 

/**
  * @}
  */

/*-----------------------------------------------------------------------------------------------*/
/* Private functions                                                                             */
/*-----------------------------------------------------------------------------------------------*/
/** @defgroup ble_private_functions Private functions
  * @{
  */

// callback //Callback pour gérer les événements GAP
static int gap_event_handler(struct ble_gap_event *event, void *arg) {

    switch (event->type) {

        //  SCAN 
        case BLE_GAP_EVENT_DISC:
            if (current_role == BSP_BLE_ROLE_CENTRAL) {
            ESP_LOGI("BLE_APP", "pub recue : addr=%02x:%02x:%02x:%02x:%02x:%02x, rssi=%d",
                     event->disc.addr.val[5], event->disc.addr.val[4],
                     event->disc.addr.val[3], event->disc.addr.val[2],
                     event->disc.addr.val[1], event->disc.addr.val[0],
                     event->disc.rssi);

            
                if (memcmp(event->disc.addr.val, target_addr, 6) == 0)
                { 
                    ESP_LOGI(TAG, "Adresse trouvee, pret pour  connexion...");
                     bsp_ble_scan_stop();

                    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC,
                                             &event->disc.addr,
                                             50000,
                                             &conn_params,
                                             gap_event_handler,
                                             NULL);
                    if (rc != 0) {
                        ESP_LOGE(TAG, "Erreur connexion : %d", rc);
                    }
                    else {
                           ESP_LOGI(TAG, "Connexion en cours...");
                          }
                }
            }
            break;


        //  CONNEXION 
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "Connecte  ");
                  if (current_role == BSP_BLE_ROLE_CENTRAL) {
                     bsp_ble_central_write(0x000C, "hello coucou");
        ESP_LOGI(TAG, "Valeur envoyée après connexion");

        vTaskDelay(500 / portTICK_PERIOD_MS);
                  
        bsp_ble_central_read(0x000C);
        ESP_LOGI(TAG, " Lecture effectuée après connexion");
                  }
            } 
            else {
                ESP_LOGE(TAG, "echec connexion");
                
                if (current_role == BSP_BLE_ROLE_CENTRAL) {
                    ble_gap_disc(ble_addr_type, BLE_HS_FOREVER, &current_disc_params, gap_event_handler, NULL);
                } else if (current_role == BSP_BLE_ROLE_PERIPHERAL) {
                    bsp_ble_start_adv();
                }
            }
            break;

        // DECONNEXION 
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Deconnecte ");
            conn_handle = 0xFFFF;

            if (current_role == BSP_BLE_ROLE_CENTRAL) {
                ble_gap_disc(ble_addr_type, BLE_HS_FOREVER, &current_disc_params, gap_event_handler, NULL);
            } else if (current_role == BSP_BLE_ROLE_PERIPHERAL) {
                bsp_ble_start_adv();
            }
            break;

            // UPDATE CONNECTION  


case BLE_GAP_EVENT_CONN_UPDATE: {
    if (event->conn_update.status == 0) {
        struct ble_gap_conn_desc desc;
        int rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        if (rc == 0) {
            ESP_LOGI(TAG, "Conn params updated: interval=%d latency=%d timeout=%d",
                     desc.conn_itvl,
                     desc.conn_latency,
                     desc.supervision_timeout);
        } else {
            ESP_LOGE(TAG, "Erreur: impossible de récupérer la description de connexion");
        }
    } else {
        ESP_LOGE(TAG, "Echec de mise à jour params, status=%d",
                 event->conn_update.status);
    }
    break;
}


        //  ADV TERMINATE
        case BLE_GAP_EVENT_ADV_COMPLETE:
    if (current_role == BSP_BLE_ROLE_PERIPHERAL) {
        ESP_LOGI(TAG, "Advertising termine, stop ,relance...");
        bsp_ble_adv_stop();     
        bsp_ble_start_adv();    
    }
    break;

        default:
            break;
    }

    return 0;
} 

// Callback pour gérer les accès GATT  
static int my_access_cb(uint16_t conn_handle_cb, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg) {
    uint16_t indices = (uint16_t)(uintptr_t)arg;
    uint8_t service_idx = (indices >> 8) & 0xFF;
    uint8_t char_idx    = indices & 0xFF;

 
  
    char_handles[service_idx][char_idx] = attr_handle;

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        char_value_len[service_idx][char_idx] = OS_MBUF_PKTLEN(ctxt->om);
        if (char_value_len[service_idx][char_idx] >= MAX_VALUE_LEN) {
            char_value_len[service_idx][char_idx] = MAX_VALUE_LEN - 1;
        }

        os_mbuf_copydata(ctxt->om, 0, char_value_len[service_idx][char_idx],
                         char_values[service_idx][char_idx]);
        char_values[service_idx][char_idx][char_value_len[service_idx][char_idx]] = '\0';

        ESP_LOGI(TAG, "WRITE svc[%d] char[%d] handle=0x%04X = %s",
                 service_idx, char_idx, attr_handle, char_values[service_idx][char_idx]);

        // nvs
        if (my_nvs_handle != 0) {
            char key[32];
            snprintf(key, sizeof(key), "svc%d_chr%d", service_idx, char_idx);
            esp_err_t err = nvs_set_str(my_nvs_handle, key,
                                        (char *)char_values[service_idx][char_idx]);
            if (err == ESP_OK) {
                nvs_commit(my_nvs_handle);
                ESP_LOGI(TAG, "Saved to NVS key=%s", key);
            } else {
                ESP_LOGE(TAG, " NVS set_str failed (%d) for key=%s", err, key);
            }
        }

        return 0;
    } 
    else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        if (char_value_len[service_idx][char_idx] > 0) {
            os_mbuf_append(ctxt->om, char_values[service_idx][char_idx],
                           char_value_len[service_idx][char_idx]);
        }

        ESP_LOGI(TAG, "READ svc[%d] char[%d] handle=0x%04X = %s",
                 service_idx, char_idx, attr_handle, char_values[service_idx][char_idx]);

        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}


//callback si ble pret 
static void ble_app_on_sync(void) {

    ble_hs_id_infer_auto(0, &ble_addr_type);

    
    bsp_ble_set_role(BSP_BLE_ROLE_PERIPHERAL); 
    bsp_ble_get_role();


    if (current_role == BSP_BLE_ROLE_PERIPHERAL) {
        ESP_LOGI(TAG, "Initialisation advertising en mode PERIPHERAL");
        bsp_ble_set_adv_params("esp zeinab", BLE_GAP_CONN_MODE_UND, BLE_GAP_DISC_MODE_GEN);
        bsp_ble_start_adv();
        bsp_ble_get_adv_params();
        
   
    } else if (current_role == BSP_BLE_ROLE_CENTRAL) {
        ESP_LOGI(TAG, "Mode CENTRAL: pret pour le scan");
        bsp_ble_scan_start();
        bsp_ble_get_scan_params();

     bsp_ble_update_conn_params(conn_handle,
                                       0x0050,  
                                       0x00A0,  
                                       0,       
                                       400); 
             }
                                    
                                    
                         
    }



void ble_host_task(void *param) {
    nimble_port_run(); 
}

/**
  * @}
  */

/*-----------------------------------------------------------------------------------------------*/
/* Exported functions                                                                            */
/*-----------------------------------------------------------------------------------------------*/
/** @defgroup ble_exported_functions Exported functions
  * @{
  */

  /* ==============================================================================================
  *                                      role 
  * ============================================================================================== */

bsp_err_t bsp_ble_set_role(bsp_ble_role_t role)
{
    if (role != BSP_BLE_ROLE_PERIPHERAL && role != BSP_BLE_ROLE_CENTRAL) {
        ESP_LOGE(TAG, "role invalide ");
        return BSP_ERR_ARG;
    }

    current_role = role;
    ESP_LOGI(TAG, "Role BLE défini sur : %s",
             (role == BSP_BLE_ROLE_PERIPHERAL) ? "Peripheral" : "Central");

    return BSP_OK;
}

bsp_err_t bsp_ble_get_role(void)
{
    if (current_role == BSP_BLE_ROLE_PERIPHERAL)
    ESP_LOGE(TAG, "role actuel est Peripheral  " );
    else if  (current_role == BSP_BLE_ROLE_CENTRAL)
     ESP_LOGE(TAG, "role actuel est central  " );
     else 
     ESP_LOGE(TAG, "role actuel none " );
    
    return BSP_OK;
}
  /* ==============================================================================================
  *                                     connection params 
  * ============================================================================================== */

 bsp_err_t bsp_ble_update_conn_params(uint16_t conn_handle, 
                                   uint16_t itvl_min, uint16_t itvl_max,
                                   uint16_t latency, uint16_t supervision_timeout) 
{
    struct ble_gap_upd_params params = {
        .itvl_min = itvl_min,
        .itvl_max = itvl_max,
        .latency = latency,
        .supervision_timeout = supervision_timeout,
    };
    
   
    int rc = ble_gap_update_params(conn_handle, &params);
    if (rc != 0) {
        ESP_LOGE(TAG, "Erreur conn params update: %d", rc);
        return BSP_ERR;
    }
    
    ESP_LOGI(TAG, "Requete paramètres connexion envoyée: min=%d, max=%d, latency=%d, timeout=%d",
             itvl_min, itvl_max, latency, supervision_timeout);
    
    return BSP_OK;
}

  /* ==============================================================================================
  *                                     scan --- central 
  * ============================================================================================== */
bsp_err_t bsp_ble_set_scan_params(uint16_t itvl, uint16_t window,
                                 uint8_t limited, uint8_t passive,
                                 uint8_t filter_duplicates, uint8_t filter_policy)
{
    current_disc_params.itvl = itvl;
    current_disc_params.window = window;
    current_disc_params.limited = limited;
    current_disc_params.passive = passive;
    current_disc_params.filter_duplicates = filter_duplicates;
    current_disc_params.filter_policy = filter_policy;

    return BSP_OK;  
}


bsp_err_t bsp_ble_get_scan_params(void)
{
    ESP_LOGI(TAG, "Scan params: itvl=%d, window=%d, limited=%d, passive=%d, filter_dup=%d, policy=%d",
             current_disc_params.itvl, 
             current_disc_params.window, 
             current_disc_params.limited,
             current_disc_params.passive, 
             current_disc_params.filter_duplicates,
             current_disc_params.filter_policy);

    return BSP_OK;
}


bsp_err_t bsp_ble_scan_start(void) {
    if (current_role != BSP_BLE_ROLE_CENTRAL) 
    {
        ESP_LOGE("BSP_BLE", "erreur  role != central");
        return BSP_ERR;
    }

    int rc = ble_gap_disc(ble_addr_type,
                          BLE_HS_FOREVER,     
                          &current_disc_params,
                          gap_event_handler,        
                          NULL);  

    if (rc != 0) {
        ESP_LOGE("BSP_BLE", "Erreur ble_gap_disc: %d", rc);
        return BSP_ERR;
    }

    ESP_LOGI("BSP_BLE", "Scan démarre");
    return BSP_OK;
}


bsp_err_t bsp_ble_scan_stop(void) {
    if (current_role != BSP_BLE_ROLE_CENTRAL) {
        ESP_LOGE("BSP_BLE", "erreur role != central");
        return BSP_ERR;
    }

    int rc = ble_gap_disc_cancel();
    if (rc != 0) {
        ESP_LOGE("BSP_BLE", "Erreur ble_gap_disc_cancel: %d", rc);
        return BSP_ERR;
    }

    ESP_LOGI("BSP_BLE", "Scan arrête");
    return BSP_OK;
}


bsp_err_t bsp_ble_central_disconnect(void) {
   
    if (current_role != BSP_BLE_ROLE_CENTRAL) {
        ESP_LOGW(TAG, "Deconnexion impossible : role != central ");
        return  BSP_ERR_VAL  ;
    }

    
    if (conn_handle == 0xFFFF) {  
        ESP_LOGW(TAG, "Aucune connexion active !");
        return BSP_ERR_NO_DATA;
    }

   
    int rc = ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);  
    if (rc != 0) {
        ESP_LOGE(TAG, "Erreur de deconnexion : %d", rc);
        return BSP_ERR;
    }

    ESP_LOGI(TAG, "Deconnexion demandé en mode central ");
    conn_handle = 0xFFFF;  
    return BSP_OK;
}    


bsp_err_t bsp_ble_central_write(uint16_t handle, const char *data) {
    if (conn_handle == 0xFFFF) {
        ESP_LOGW("BSP_BLE_SERVICE", "Pas de connexion active pour écrire");
        return BSP_ERR_NO_DATA;
    }

    int rc = ble_gattc_write_flat(conn_handle, handle, data, strlen(data), NULL, NULL);
    if (rc == 0) {
        ESP_LOGI("BSP_BLE_SERVICE", "WRITE envoyé: %s (handle=0x%04X)", data, handle);
        return BSP_OK;
    } else {
        ESP_LOGE("BSP_BLE_SERVICE", "Erreur WRITE: %d", rc);
        return BSP_ERR;  
    }
}


bsp_err_t bsp_ble_central_read(uint16_t handle) {
    if (conn_handle == 0xFFFF) {
        ESP_LOGW("BSP_BLE_SERVICE", "Pas de connexion active pour lire");
        return BSP_ERR_NO_DATA;
    }

    int rc = ble_gattc_read(conn_handle, handle, NULL, NULL);
    if (rc == 0) {
        ESP_LOGI("BSP_BLE_SERVICE", "READ demandé (handle=0x%04X)", handle);
        return BSP_OK;
    } else {
        ESP_LOGE("BSP_BLE_SERVICE", "Erreur READ: %d", rc);
        return BSP_ERR;
    }
}


 /* ==============================================================================================
  *                                     advertising --- peripheral 
  * ============================================================================================== */


bsp_err_t bsp_ble_set_adv_params(const char *device_name,uint8_t conn_mode,uint8_t disc_mode) {
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    if (device_name == NULL || strlen(device_name) == 0) {
        ESP_LOGE(TAG, "rentrer le nom ");
        return BSP_ERR_VAL;
    }

     strncpy(g_device_name, device_name, sizeof(g_device_name) - 1);
    g_conn_mode = conn_mode;
    g_disc_mode = disc_mode;

    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Erreur configuration de advertising fields : %d", rc);
        return BSP_ERR_VAL;
    }

       g_adv_params.conn_mode = conn_mode;
       g_adv_params.disc_mode = disc_mode; 

    ESP_LOGI(TAG, "Paramètres BLE configurés : name=\"%s\", conn_mode=%d, disc_mode=%d",
             device_name, conn_mode, disc_mode);

    return BSP_OK;
}


bsp_err_t bsp_ble_get_adv_params(void) {
    ESP_LOGI(TAG, "Nom  : %s", g_device_name);
    ESP_LOGI(TAG, "Conn mode  : %d", g_conn_mode);
    ESP_LOGI(TAG, "Disc mode  : %d", g_disc_mode);
    return BSP_OK;
}


bsp_err_t bsp_ble_start_adv(void) {
     if (current_role != BSP_BLE_ROLE_PERIPHERAL) {
        ESP_LOGE("BSP_BLE", "erreur  : role != peripheral");
        return BSP_ERR;
    }
    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                               &g_adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Erreur demarrage advertising : %d", rc);
        return BSP_ERR;
    }

    ESP_LOGI(TAG, "Advertising BLE démarre !");
    return BSP_OK;
}

bsp_err_t bsp_ble_adv_stop(void)
{
    int rc = ble_gap_adv_stop();

    if (rc == 0) {
        ESP_LOGI(TAG, "Advertising arrêté ");
        return BSP_OK;
    } else {
        ESP_LOGE(TAG, "Erreur arrêt advertising : %d", rc);
        return BSP_ERR;
    }
}


bsp_err_t bsp_ble_peripheral_disconnect(void) {
    
    if (current_role != BSP_BLE_ROLE_PERIPHERAL) {
        ESP_LOGW(TAG, "Deconnexion impossible : role != peripheral");
        return  BSP_ERR_VAL  ;
    }

    
    if (conn_handle == 0xFFFF) {  
        ESP_LOGW(TAG, "Aucune connexion active !");
        return BSP_ERR_NO_DATA;
    }

   
    int rc = ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);  
    if (rc != 0) {
        ESP_LOGE(TAG, "Erreur de deconnexion : %d", rc);
        return BSP_ERR;
    }

    ESP_LOGI(TAG, "Deconnexion demandé en mode peripheral");
    conn_handle = 0xFFFF;  
    return BSP_OK;
}

 /* ==============================================================================================
  *                                     services & caracteristics
  * ============================================================================================== */



bsp_err_t bsp_ble_add_service(uint16_t service_uuid) {
    if (service_count >= MAX_SERVICES) {
        return BSP_ERR_ARG;
    }

    service_uuids[service_count] = (ble_uuid16_t)BLE_UUID16_INIT(service_uuid);

  
    my_services[service_count] = (struct ble_gatt_svc_def){
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (ble_uuid_t *)&service_uuids[service_count],
        .characteristics = my_characteristics[service_count],
    };

    service_count++;
    my_services[service_count] = (struct ble_gatt_svc_def){0}; 
    return BSP_OK;
}



bsp_err_t bsp_ble_add_characteristic(int service_index, uint16_t char_uuid, uint8_t flags) {
    if (service_index >= service_count || char_count[service_index] >= MAX_CHARS_PER_SERVICE) {
        return BSP_ERR_NO_DATA;
    }

    int char_idx = char_count[service_index];

    char_uuids[service_index][char_idx].u.type = BLE_UUID_TYPE_16;
    char_uuids[service_index][char_idx].value = char_uuid;

    my_characteristics[service_index][char_idx] = (struct ble_gatt_chr_def){
        .uuid = (ble_uuid_t *)&char_uuids[service_index][char_idx],
        .access_cb = my_access_cb,
        .arg = (void *)(uintptr_t)((service_index << 8) | char_idx),
        .flags = flags,
    };

    char_count[service_index]++;
    my_characteristics[service_index][char_count[service_index]] = (struct ble_gatt_chr_def){0};

    return BSP_OK;
}



bsp_err_t bsp_ble_read_characteristic(struct ble_gatt_chr_def *chr, uint16_t uuid16) {
    if (read_index >= 4) return BSP_ERR_NO_DATA; 
    g_uuids_read[read_index] = (ble_uuid16_t){
        .u = { .type = BLE_UUID_TYPE_16 },
        .value = uuid16
    };

    *chr = (struct ble_gatt_chr_def){
        .uuid = (ble_uuid_t *)&g_uuids_read[read_index],
        .access_cb = my_access_cb,
        .flags = BLE_GATT_CHR_F_READ,
    };
    read_index++;
    return BSP_OK;
}



bsp_err_t bsp_ble_write_characteristic(struct ble_gatt_chr_def *chr, uint16_t uuid16) {
    if (write_index >= 4) return BSP_ERR_NO_DATA;
    g_uuids_write[write_index] = (ble_uuid16_t){
        .u = { .type = BLE_UUID_TYPE_16 },
        .value = uuid16
    };

    *chr = (struct ble_gatt_chr_def){
        .uuid = (ble_uuid_t *)&g_uuids_write[write_index],
        .access_cb = my_access_cb,
        .flags = BLE_GATT_CHR_F_WRITE,
    };
    write_index++;
    return BSP_OK;
}


bsp_err_t bsp_ble_notify_characteristic (struct ble_gatt_chr_def *chr, uint16_t uuid16) {
    if (notify_index >= 4) return BSP_ERR_NO_DATA; 

    g_uuids_notify[notify_index] = (ble_uuid16_t){
        .u = { .type = BLE_UUID_TYPE_16 },
        .value = uuid16
    };

    *chr = (struct ble_gatt_chr_def){
        .uuid = (ble_uuid_t *)&g_uuids_notify[notify_index],
        .access_cb =NULL,
        //.val_handle = NULL,
        .flags = BLE_GATT_CHR_F_NOTIFY,
    };

    notify_index++;
    return BSP_OK;
}


bsp_err_t bsp_ble_indicate_characteristic(struct ble_gatt_chr_def *chr, uint16_t uuid16) {
    if (indicate_index >= 4) return BSP_ERR_NO_DATA; 

    g_uuids_indicate[indicate_index] = (ble_uuid16_t){
        .u = { .type = BLE_UUID_TYPE_16 },
        .value = uuid16
    };

    *chr = (struct ble_gatt_chr_def){
        .uuid = (ble_uuid_t *)&g_uuids_indicate[indicate_index],
        .access_cb = NULL,
        //.val_handle = NULL,
        .flags = BLE_GATT_CHR_F_INDICATE,
    };

    indicate_index++;
    return BSP_OK;
}

/** ***********************************************************************************************
 * @brief      Initialize the ble driver (ble stack, gap and gatt services init)
 * @retval     BSP result as defined in ::bsp_err_t
  ********************************************************************************************** */
bsp_err_t bsp_ble_init(void)
{
    bsp_err_t eRetVal = BSP_ERR;

    
    if (nvs_flash_init() != ESP_OK) {
        ESP_LOGE(TAG, "Erreur init NVS");
        return BSP_ERR;
    }

 
    if (esp_nimble_hci_and_controller_init() != ESP_OK) {
        ESP_LOGE(TAG, "Erreur init HCI");
        return BSP_ERR;
    }

    if (nimble_port_init() != 0) {
        ESP_LOGE(TAG, "Erreur init NimBLE port");
        return BSP_ERR;
    }

   
    ble_svc_gap_init();
    ble_svc_gatt_init();

   
    ble_hs_cfg.sync_cb = ble_app_on_sync;      //  démarre advertising/scan
   
      ble_hs_cfg.reset_cb = NULL;

    // Lance nimble host 
    nimble_port_freertos_init(ble_host_task);
   
    ble_svc_gap_device_name_set("BSP_BLE_Device");

    ESP_LOGI(TAG, "BSP BLE init done");
    eRetVal = BSP_OK;
    return eRetVal;
}
/** ***********************************************************************************************
 * @brief      Exit the ble driver
 * @retval     BSP result as defined in ::bsp_err_t
  ********************************************************************************************** */
bsp_err_t bsp_ble_exit(void)
{
    bsp_err_t eRetVal = BSP_ERR;

    // connexion est active → déconnecter
    if (conn_handle != 0xFFFF) {
        int rc = ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        if (rc != 0) {
            ESP_LOGW(TAG, "Erreur disconnect rc=%d", rc);
        }
        conn_handle = 0xFFFF;
    }

    //  arret adv ou scan
    ble_gap_adv_stop();
    ble_gap_disc_cancel();

    // arret nimble
    nimble_port_stop();
    nimble_port_deinit();

    // arret hci 
    esp_nimble_hci_and_controller_deinit();

    ESP_LOGI(TAG, "BSP BLE exit done");
    eRetVal = BSP_OK;
    return eRetVal;
}

/** ***********************************************************************************************
 * @brief      Control the ble driver
 * @param      eCtrlId Control ID as defined in ::bsp_ctrl_t
 * @param      u8Argc Arguments count
 * @param      pArgv Arguments pointers list
 * @return     BSP result as defined in ::bsp_err_t
  ********************************************************************************************** */
bsp_err_t bsp_ble_ctrl(bsp_ctrl_t eCtrlId, uint8_t u8Argc, void** pArgv)
{
  bsp_err_t eRetVal;

  switch(eCtrlId)
  {
  case BSP_COM_BLE_ADV_START:
            if(u8Argc == 0) {
                eRetVal = bsp_ble_start_adv();
            } else {
                eRetVal = BSP_ERR_ARG;
            }
            break;

 case BSP_COM_BLE_ADV_STOP:
            if(u8Argc == 0) {
                eRetVal = bsp_ble_adv_stop();
            } else {
                eRetVal = BSP_ERR_ARG;
            }
            break;

  case BSP_COM_BLE_ADD_SERVICE:
            if(u8Argc == 1 && pArgv && pArgv[0]) {
                uint16_t service_uuid = *((uint16_t*)pArgv[0]);
                eRetVal = bsp_ble_add_service(service_uuid);
            } else {
                eRetVal = BSP_ERR_ARG;
            }
            break;
            
  case BSP_COM_BLE_ADD_CARACTERISTIC:
     if(u8Argc == 3 && pArgv && pArgv[0] && pArgv[1] && pArgv[2]) {
                int service_index = *((int*)pArgv[0]);
                uint16_t char_uuid = *((uint16_t*)pArgv[1]);
                uint8_t flags = *((uint8_t*)pArgv[2]);
                eRetVal = bsp_ble_add_characteristic(service_index, char_uuid, flags);
            } else {
                eRetVal = BSP_ERR_ARG;
            }
            break; 

  case BSP_COM_BLE_READ_CARACTERISTIC:
    if(u8Argc == 2 && pArgv) {
                eRetVal = bsp_ble_read_characteristic(
                    (struct ble_gatt_chr_def*)pArgv[0],
                    *((uint16_t*)pArgv[1])
                );
            } else eRetVal = BSP_ERR_ARG;
            break;

  case BSP_COM_BLE_WRITE_CARACTERISTIC:
    if(u8Argc == 2 && pArgv) {
                eRetVal = bsp_ble_write_characteristic(
                    (struct ble_gatt_chr_def*)pArgv[0],
                    *((uint16_t*)pArgv[1])
                );
            } else eRetVal = BSP_ERR_ARG;
            break;

  case BSP_COM_BLE_ADD_NOTIFICATION:
   if(u8Argc == 2 && pArgv) {
                eRetVal = bsp_ble_notify_characteristic(
                    (struct ble_gatt_chr_def*)pArgv[0],
                    *((uint16_t*)pArgv[1])
                );
            } else eRetVal = BSP_ERR_ARG;
            break;
  case BSP_COM_BLE_ADD_INDICATE:
     if(u8Argc == 2 && pArgv) {
                eRetVal = bsp_ble_indicate_characteristic(
                    (struct ble_gatt_chr_def*)pArgv[0],
                    *((uint16_t*)pArgv[1])
                );
            } else eRetVal = BSP_ERR_ARG;
            break;
  case BSP_COM_BLE_SET_ADV_PARAMS:
     if(u8Argc == 3 && pArgv) {
                eRetVal = bsp_ble_set_adv_params(
                    (const char*)pArgv[0],
                    *((uint8_t*)pArgv[1]),
                    *((uint8_t*)pArgv[2])
                );
            } else eRetVal = BSP_ERR_ARG;
            break;

  case BSP_COM_BLE_GET_ADV_PARAMS:
    eRetVal = bsp_ble_get_adv_params();
            break;

  case BSP_COM_BLE_UPDATE_CONN_PARAMS:
     if(u8Argc == 5 && pArgv) {
                uint16_t conn_handle = *((uint16_t*)pArgv[0]);
                uint16_t itvl_min = *((uint16_t*)pArgv[1]);
                uint16_t itvl_max = *((uint16_t*)pArgv[2]);
                uint16_t latency = *((uint16_t*)pArgv[3]);
                uint16_t timeout = *((uint16_t*)pArgv[4]);
                eRetVal = bsp_ble_update_conn_params(conn_handle, itvl_min, itvl_max, latency, timeout);
            } else {
                eRetVal = BSP_ERR_ARG;
            }
            break;

  case BSP_COM_BLE_PERIPHERAL_DISCONNECT:
    eRetVal = bsp_ble_peripheral_disconnect();
            break;

  case BSP_BLE_SET_ROLE:
     if(u8Argc == 1 && pArgv && pArgv[0]) {
                bsp_ble_role_t role = *((bsp_ble_role_t*)pArgv[0]);
                eRetVal = bsp_ble_set_role(role);
            } else {
                eRetVal = BSP_ERR_ARG;
            }
            break;

  case BSP_BLE_GET_ROLE:
     eRetVal = bsp_ble_get_role();
            break;

  case BSP_COM_BLE_SET_SCAN_PARAMS:
    if(u8Argc == 6 && pArgv) {
                eRetVal = bsp_ble_set_scan_params(
                    *((uint16_t*)pArgv[0]),
                    *((uint16_t*)pArgv[1]),
                    *((uint8_t*)pArgv[2]),
                    *((uint8_t*)pArgv[3]),
                    *((uint8_t*)pArgv[4]),
                    *((uint8_t*)pArgv[5])
                );
            } else eRetVal = BSP_ERR_ARG;
            break;

  case BSP_COM_BLE_GET_SCAN_PARAMS:
    eRetVal = bsp_ble_get_scan_params();
            break;
            
  case BSP_COM_BLE_SCAN_START:
    eRetVal = bsp_ble_scan_start();
            break;

  case BSP_COM_BLE_SCAN_STOP:
    eRetVal = bsp_ble_scan_stop();
            break;

  case BSP_COM_BLE_CENTRAL_DISCONNECT:
    eRetVal = bsp_ble_central_disconnect();
            break;
  case BSP_COM_BLE_CENTRAL_WRITE:
            if(u8Argc == 2 && pArgv) {
                eRetVal = bsp_ble_central_write(*((uint16_t*)pArgv[0]), (const char*)pArgv[1]);
            } else eRetVal = BSP_ERR_ARG;
            break;

  case BSP_COM_BLE_CENTRAL_READ:
            if(u8Argc == 1 && pArgv) {
                eRetVal = bsp_ble_central_read(*((uint16_t*)pArgv[0]));
            } else eRetVal = BSP_ERR_ARG;
            break;

  default:
    eRetVal = BSP_ERR_VAL;
    break;
  }
  return eRetVal;
}

/**
  * @}
  */


/**
  * @}
  */
