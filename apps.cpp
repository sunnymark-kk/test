/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#include "stdio.h"
#include "cmsis_os.h"
#include "factory_section.h"
#include "list.h"
#include "string.h"

#include "hal_timer.h"
#include "hal_trace.h"
#include "hal_bootmode.h"

#include "audioflinger.h"
#include "apps.h"
#include "app_thread.h"
#include "app_key.h"
#include "app_pwl.h"
#include "app_audio.h"
#include "app_overlay.h"
#include "app_battery.h"
#include "app_utils.h"
#include "app_status_ind.h"
#ifdef __FACTORY_MODE_SUPPORT__
#include "app_factory.h"
#include "app_factory_bt.h"
#endif
#include "bt_drv_interface.h"
#include "besbt.h"
#include "norflash_api.h"
#include "nvrecord.h"
#include "nvrecord_env.h"
#include "nvrecord_extension.h"

#include "me_api.h"
#include "a2dp_api.h"
#include "os_api.h"
#include "btapp.h"
#include "app_bt.h"
#include "apps_key_map.h"
#ifdef __INTERCONNECTION__
#include "app_interconnection.h"
#include "app_interconnection_logic_protocol.h"
#include "app_ble_mode_switch.h"
#endif

#ifdef __PC_UART_MOD_CUSTOM__
#include "uart_cmd.h"
#include "hal_cmd.h"
#endif

#if defined(HALFDUPLEXUART)
#include "halfduplexuart.h"
#endif
#if GSOUND_OTA_ENABLED
#include "gsound_ota.h"
#endif
#ifdef BES_OTA_BASIC
#include "ota_bes.h"
#endif
#ifdef MEDIA_PLAYER_SUPPORT
#include "resources.h"
#include "app_media_player.h"
#endif
#include "app_bt_media_manager.h"
#include "hal_sleep.h"
#if VOICE_DATAPATH
#include "app_voicepath.h"
#endif

#if defined(__FORCE_OTABOOT_UPDATE__)
#include "apps_ota_checker.h"
#endif

#ifdef __THIRDPARTY
#include "app_thirdparty.h"
#endif

#ifdef AUDIO_DEBUG_V0_1_0
extern "C" int speech_tuning_init(void);
#endif

#include "app_ble_cmd_handler.h"
#include "app_ble_custom_cmd.h"
#include "rwapp_config.h"

#ifdef __TWS__
#include "app_tws.h"
#include "app_tws_if.h"
#include "app_tws_role_switch.h"
#include "app_tws_ui.h"
#endif
#include "app.h"
#ifdef ANC_APP
#include "app_anc.h"
#endif

#if defined(TWS_RBCODEC_PLAYER) || defined(TWS_LINEIN_PLAYER)
void player_role_thread_init(void);
#endif

#include "app_bt_conn_mgr.h"
#include "ble_tws.h"
#include "analog.h"
#include "log_section.h"
#include "app_hfp.h"
#include "app_spp.h"

#ifdef __AI_VOICE__
#include "ai_manager.h"
#include "ai_thread.h"
#include "ai_control.h"
#include "ai_spp.h"
#include "app_ai_voice.h"
#endif

#include "gapm_task.h"

#ifdef TEST_OVER_THE_AIR_ENANBLED
#include "app_tota.h"
#endif

#if _GFPS_
#include "app_gfps.h"
#include "app_fp_rfcomm.h"
#endif

#define LOG_MODULE HAL_TRACE_MODULE_APP

extern "C" {
void hal_trace_crashdump_print(void);
void app_fast_pair_timeout_handler(void);
}

#define APP_BATTERY_LEVEL_LOWPOWERTHRESHOLD (1)

static uint8_t app_system_status = APP_SYSTEM_STATUS_INVALID;

uint8_t app_system_status_set(int flag)
{
    uint8_t old_status = app_system_status;
    app_system_status  = flag;
    return old_status;
}

uint8_t app_system_status_get(void)
{
    return app_system_status;
}

#ifdef MEDIA_PLAYER_RBCODEC
extern int  rb_ctl_init();
extern bool rb_ctl_is_init_done(void);
extern void app_rbplay_audio_reset_pause_status(void);
#endif

#ifdef __COWIN_V2_BOARD_ANC_SINGLE_MODE__
extern bool app_pwr_key_monitor_get_val(void);
static bool anc_single_mode_on = false;

#if defined(BTUSB_AUDIO_MODE)
static bool app_usbaudio_mode = false;
void app_usbaudio_entry(void)
{
    app_usbaudio_mode = true;

    btusbaudio_entry();
}

bool app_usbaudio_mode_on(void)
{
    return app_usbaudio_mode;
}
#endif

extern "C" bool anc_single_mode_is_on(void)
{
    return anc_single_mode_on;
}
#endif

#ifndef APP_TEST_MODE
static uint8_t app_status_indication_init(void)
{
    struct APP_PWL_CFG_T cfg;
    memset(&cfg, 0, sizeof(struct APP_PWL_CFG_T));
    app_pwl_open();
    app_pwl_setup(APP_PWL_ID_0, &cfg);
    app_pwl_setup(APP_PWL_ID_1, &cfg);
    return 0;
}
#endif

void app_refresh_random_seed(void)
{
    uint32_t generatedSeed = hal_sys_timer_get();
    for (uint8_t index = 0; index < sizeof(bt_addr); index++)
    {
        generatedSeed ^= (((uint32_t)(bt_addr[index])) << (hal_sys_timer_get() & 0xF));
    }
    srand(generatedSeed);
}
#if defined(__BTIF_EARPHONE__) && defined(__BTIF_AUTOPOWEROFF__)

void PairingTransferToConnectable(void);

typedef void (*APP_10_SECOND_TIMER_CB_T)(void);

void app_pair_timerout(void);
void app_poweroff_timerout(void);
void CloseEarphone(void);

typedef struct {
    uint8_t timer_id;
    uint8_t timer_en;
    uint8_t timer_count;
    uint8_t timer_period;

    APP_10_SECOND_TIMER_CB_T cb;
} APP_10_SECOND_TIMER_STRUCT;

static APP_10_SECOND_TIMER_STRUCT app_pair_timer = {
    .timer_id     = APP_PAIR_TIMER_ID,
    .timer_en     = 0,
    .timer_count  = 0,
    .timer_period = 6,
    .cb           = PairingTransferToConnectable
};

extern void app_reconnect_timeout_handle(void);

static APP_10_SECOND_TIMER_STRUCT app_reconnect_timer = {
    .timer_id     = APP_RECONNECT_TIMER_ID,
    .timer_en     = 0,
    .timer_count  = 0,
    .timer_period = 120,
    .cb           = app_reconnect_timeout_handle
};

static APP_10_SECOND_TIMER_STRUCT app_poweroff_timer = {
    .timer_id     = APP_POWEROFF_TIMER_ID,
    .timer_en     = 0,
    .timer_count  = 0,
    .timer_period = 180,
    .cb           = CloseEarphone
};

#if _GFPS_
static APP_10_SECOND_TIMER_STRUCT app_fastpair_timer = {
    .timer_id     = APP_FAST_PAIR_LASTING_TIMER_ID,
    .timer_en     = 0,
    .timer_count  = 0,
    .timer_period = APP_FAST_PAIRING_TIMEOUT_IN_SECOND / 10,
    .cb           = app_fast_pair_timeout_handler
};
#endif

static APP_10_SECOND_TIMER_STRUCT *app_10_second_array[] = {
    &app_pair_timer,
    &app_poweroff_timer,
    &app_reconnect_timer,
#if _GFPS_
    &app_fastpair_timer,
#endif
};

void app_stop_10_second_timer(uint8_t timer_id)
{
    APP_10_SECOND_TIMER_STRUCT *timer;

    if (timer_id >= ARRAY_SIZE(app_10_second_array)) {
        ASSERT(0, "timer id error");
        return;
    }
    timer = app_10_second_array[timer_id];
    timer->timer_en                   = 0;
    timer->timer_count                = 0;
}

void app_start_10_second_timer(uint8_t timer_id)
{
    APP_10_SECOND_TIMER_STRUCT *timer;

    if (timer_id >= ARRAY_SIZE(app_10_second_array)) {
        ASSERT(0, "timer id error");
        return;
    }
    timer = app_10_second_array[timer_id];
    timer->timer_en    = 1;
    timer->timer_count = 0;
}

static void app_10_second_timerout_handle(uint8_t timer_id)
{
    APP_10_SECOND_TIMER_STRUCT *timer;

    if (timer_id >= APP_10_SECOND_TIMER_TOTAL_NUM) {
        ASSERT(0, "timer id error");
        return;
    }
    timer = app_10_second_array[timer_id];
    timer->timer_en = 0;
    timer->cb();
}

static void app_10_second_timer_check(void)
{
    uint8_t i;

    for (i = 0; i < ARRAY_SIZE(app_10_second_array); i++) {
        if (app_10_second_array[i]->timer_en) {
            app_10_second_array[i]->timer_count++;
            if (app_10_second_array[i]->timer_count >= app_10_second_array[i]->timer_period) {
                app_10_second_timerout_handle(i);
            }
        }
    }
}
#endif

void       intersys_sleep_checker(void);
extern int app_bt_stream_local_volume_get(void);
extern int voicebtpcm_pcm_msbc_sync_offset_get_local(uint16_t *msbc_sync_offset);

static void app_peridic_ble_activity_check(void)
{
#ifdef __ENABLE_IN_BOX_STATE_HANDLE__
    if (!app_is_charger_box_closed())
#endif
    {
        if (app_tws_is_master_mode() &&
            IS_CONNECTED_WITH_MOBILE() &&
            !IS_CONNECTED_WITH_TWS() &&
            TWS_BLE_STATE_IDLE != app_tws_get_env()->state)
        {

            app_start_custom_function_in_bt_thread(0,
                                                   0,
                                                   ( uint32_t )app_tws_restart_operation);
        }
    }
}

static void app_env_checker_handler(void const *param)
{
    APP_MESSAGE_BLOCK msg;
    msg.mod_id = APP_MODUAL_ENVCHECKER;
    app_mailbox_put(&msg);
}

static int app_env_checker_process(APP_MESSAGE_BODY *msg_body)
{
    {
        a2dp_stream_t *         sink_stream   = app_tws_get_sink_stream();
        a2dp_stream_t *         source_stream = app_tws_get_tws_source_stream();
        struct btdevice_volume *dev_vol_p     = app_bt_stream_volume_get_ptr();
        int                     local_vol     = app_bt_stream_local_volume_get();
        int                     msbc_sync_valid;
        uint16_t                msbc_sync_offset;

        btif_remote_device_t *     sink_remDev   = NULL;
        btif_remote_device_t *     source_remDev = NULL;
        struct AF_STREAM_CONFIG_T *stream_cfg    = NULL;
        af_stream_get_cfg(AUD_STREAM_ID_0, AUD_STREAM_PLAYBACK, &stream_cfg, false);

        msbc_sync_valid = voicebtpcm_pcm_msbc_sync_offset_get_local(&msbc_sync_offset);

        sink_remDev   = btif_a2dp_get_remote_device(sink_stream);
        source_remDev = btif_a2dp_get_remote_device(source_stream);
#ifdef __ENABLE_IN_BOX_STATE_HANDLE__
        LOG_DEBUG("inbox %d boxOpened %d connMobile:%x connTws:%x mode:%d roleproc:%d gainadj:%d ble:%d",
                      app_is_in_charger_box(),
                      !app_is_charger_box_closed(),
                      CURRENT_MOBILE_DEV_CONN_STATE(),
                      CURRENT_TWS_CONN_STATE(),
                      app_tws_get_mode(),
                      app_tws_is_role_switching_on(),
                      btdrv_rf_rx_gain_adjust_getstatus(),
                      app_tws_get_env()->state);
#else
        LOG_DEBUG("connMobile:%x connTws:%x mode:%d roleproc:%d gainadj:%d ble:%d",
                      CURRENT_MOBILE_DEV_CONN_STATE(),
                      CURRENT_TWS_CONN_STATE(),
                      app_tws_get_mode(),
                      app_tws_is_role_switching_on(),
                      btdrv_rf_rx_gain_adjust_getstatus(),
                      app_tws_get_env()->state);
#endif

        if (sink_stream)
        {
            LOG_DEBUG("SINK STATE=%x", btif_a2dp_get_stream_state(sink_stream));
        }

        LOG_DEBUG("sink:0x%08x %02x/%02x/%02x source:%08x %02x/%02x/%02x",
                      sink_remDev,
                      btif_me_get_remote_device_state(sink_remDev),
                      btif_me_get_current_role(sink_remDev),
                      btif_me_get_current_mode(sink_remDev),
                      source_remDev,
                      btif_me_get_remote_device_state(source_remDev),
                      btif_me_get_current_role(source_remDev),
                      btif_me_get_current_mode(source_remDev));

        LOG_DEBUG("env: usefreq:%d/%d volume loc: %d, a2dp:%d hfp:%d real:%d msbc_offset:%d",
                      hal_sysfreq_get(),
                      hal_cmu_sys_get_freq(),
                      local_vol,
                      dev_vol_p ? dev_vol_p->a2dp_vol : 0xff,
                      dev_vol_p ? dev_vol_p->hfp_vol : 0xff,
                      stream_cfg ? stream_cfg->vol : 0xff,
                      !msbc_sync_valid ? msbc_sync_offset : 0xff);
    }

    app_start_custom_function_in_bt_thread(0,
                                           0,
                                           ( uint32_t )app_peridic_ble_activity_check);
    intersys_sleep_checker();
    osapi_notify_evm();
    return 0;
}

#define APP_ENV_CHECKER_INTERVAL_MS (7000)
osTimerDef(APP_ENV_CHECKER, app_env_checker_handler);
static osTimerId app_env_checker_timer = NULL;

int app_status_battery_report(uint8_t level)
{
    intersys_sleep_checker();
#if DEBUG_LOG_ENABLED || USER_STATISTIC_DATA_LOG_ENABLED
    log_update_time_stamp();
#endif

#ifdef __COWIN_V2_BOARD_ANC_SINGLE_MODE__
    if (anc_single_mode_on)  //anc power on,anc only mode
        return 0;
#endif
#if defined(__BTIF_EARPHONE__)
    //  app_bt_state_checker();
    app_10_second_timer_check();
#endif
    if (level <= APP_BATTERY_LEVEL_LOWPOWERTHRESHOLD)
    {
        //add something
    }
#if defined(SUPPORT_BATTERY_REPORT) || defined(SUPPORT_HF_INDICATORS)
    app_start_custom_function_in_bt_thread((uint32_t)level, 0, (uint32_t)app_hfp_battery_report);
#else
    LOG_DEBUG("[%s] Can not enable HF_CUSTOM_FEATURE_BATTERY_REPORT", __func__);
#endif
    return 0;
}

#ifdef MEDIA_PLAYER_SUPPORT

void app_status_set_num(const char *p)
{
    media_Set_IncomingNumber(p);
}

#if defined(HALFDUPLEXUART)
void usr_set_gpio(enum HAL_GPIO_PIN_T io_pin, bool set_flag)
{
    struct HAL_IOMUX_PIN_FUNCTION_MAP pinmux_usr_set_gpio[1] = {
        {( HAL_IOMUX_PIN_T )io_pin, HAL_IOMUX_FUNC_AS_GPIO, HAL_IOMUX_PIN_VOLTAGE_VIO, HAL_IOMUX_PIN_PULLUP_ENALBE},
    };

    hal_iomux_init(pinmux_usr_set_gpio, ARRAY_SIZE(pinmux_usr_set_gpio));
    hal_gpio_pin_set_dir(io_pin, HAL_GPIO_DIR_IN, 1);
    if (set_flag)
        hal_gpio_pin_set(io_pin);
    else
        hal_gpio_pin_clr(io_pin);
}
#endif

int app_voice_report_handler(APP_STATUS_INDICATION_T status, uint8_t device_id, uint8_t isMerging)
{
#if defined(BTUSB_AUDIO_MODE)
    if (app_usbaudio_mode_on())
        return 0;
#endif

#if defined(TEST_OVER_THE_AIR_ENANBLED)
    if (app_is_in_tota_mode())
    {
        LOG_DEBUG("skip voice_report when TOTA");
        return 0;
    }
#endif

    LOG_DEBUG("%s %d", __func__, status);
    AUD_ID_ENUM id = MAX_RECORD_NUM;
#ifdef __COWIN_V2_BOARD_ANC_SINGLE_MODE__
    if (anc_single_mode_on)
        return 0;
#endif
    switch (status)
    {
    case APP_STATUS_INDICATION_POWERON:
        id = AUD_ID_POWER_ON;
        break;
    case APP_STATUS_INDICATION_POWEROFF:
        id = AUD_ID_POWER_OFF;
        break;
    case APP_STATUS_INDICATION_CONNECTED:
        id = AUD_ID_BT_CONNECTED;
        break;
    case APP_STATUS_INDICATION_DISCONNECTED:
        id = AUD_ID_BT_DIS_CONNECT;
        break;
    case APP_STATUS_INDICATION_CALLNUMBER:
        id = AUD_ID_BT_CALL_INCOMING_NUMBER;
        break;
    case APP_STATUS_INDICATION_CHARGENEED:
        id = AUD_ID_BT_CHARGE_PLEASE;
        break;
    case APP_STATUS_INDICATION_FULLCHARGE:
        id = AUD_ID_BT_CHARGE_FINISH;
        break;
    case APP_STATUS_INDICATION_PAIRSUCCEED:
        id = AUD_ID_BT_PAIRING_SUC;
        break;
    case APP_STATUS_INDICATION_PAIRFAIL:
        id = AUD_ID_BT_PAIRING_FAIL;
        break;

    case APP_STATUS_INDICATION_HANGUPCALL:
        id = AUD_ID_BT_CALL_HUNG_UP;
        break;

    case APP_STATUS_INDICATION_REFUSECALL:
        id = AUD_ID_BT_CALL_REFUSE;
        break;

    case APP_STATUS_INDICATION_ANSWERCALL:
        id = AUD_ID_BT_CALL_ANSWER;
        break;

    case APP_STATUS_INDICATION_CLEARSUCCEED:
        id = AUD_ID_BT_CLEAR_SUCCESS;
        break;

    case APP_STATUS_INDICATION_CLEARFAIL:
        id = AUD_ID_BT_CLEAR_FAIL;
        break;
    case APP_STATUS_INDICATION_INCOMINGCALL:
        id = AUD_ID_BT_CALL_INCOMING_CALL;
        break;
    case APP_STATUS_INDICATION_BOTHSCAN:
        id = AUD_ID_BT_PAIR_ENABLE;
        break;
    case APP_STATUS_INDICATION_WARNING:
        id = AUD_ID_BT_WARNING;
        break;
#ifdef __TWS__
    case APP_STATUS_INDICATION_TWS_ISMASTER:
        id = AUD_ID_BT_TWS_ISMASTER;
        break;
    case APP_STATUS_INDICATION_TWS_ISSLAVE:
        id = AUD_ID_BT_TWS_ISSLAVE;
        break;
    case APP_STATUS_INDICATION_TWS_SEARCH:
        id = AUD_ID_BT_TWS_SEARCH;
        break;
    case APP_STATUS_INDICATION_TWS_STOPSEARCH:
        id = AUD_ID_BT_TWS_STOPSEARCH;
        break;
    case APP_STATUS_INDICATION_TWS_DISCOVERABLE:
        id = AUD_ID_BT_TWS_DISCOVERABLE;
        break;
    case APP_STATUS_INDICATION_TWS_LEFTCHNL:
        id = AUD_ID_BT_TWS_LEFTCHNL;
        break;
    case APP_STATUS_INDICATION_TWS_RIGHTCHNL:
        id = AUD_ID_BT_TWS_RIGHTCHNL;
        break;
#endif
#ifdef __AMA_VOICE__
    case APP_STATUS_INDICATION_ALEXA_START:
        id = AUDIO_ID_BT_ALEXA_START;
        break;
    case APP_STATUS_INDICATION_ALEXA_STOP:
        id = AUDIO_ID_BT_ALEXA_STOP;
        break;
#endif
    case APP_STATUS_INDICATION_GSOUND_MIC_OPEN:
        id = AUDIO_ID_BT_GSOUND_MIC_OPEN;
        break;
    case APP_STATUS_INDICATION_GSOUND_MIC_CLOSE:
        id = AUDIO_ID_BT_GSOUND_MIC_CLOSE;
        break;

    default:
        break;
    }

    if ((app_system_status_get() == APP_SYSTEM_STATUS_POWEROFF_PROC) &&
        (status == APP_STATUS_INDICATION_BOTHSCAN ||
         status == APP_STATUS_INDICATION_DISCONNECTED ||
         status == APP_STATUS_INDICATION_CHARGENEED ||
         status == APP_STATUS_INDICATION_CONNECTING ||
         status == APP_STATUS_INDICATION_CONNECTED))
    {
        //block ring tong
    }
	else
	{
        trigger_media_play(id, device_id, isMerging);
    }
    return 0;
}


#ifdef TWS_PROMPT_SYNC
void app_play_media_via_aud_id(uint16_t id, uint8_t device_id)
{
    app_audio_manager_sendrequest(APP_BT_STREAM_MANAGER_START,BT_STREAM_MEDIA,device_id,id,0,0);
}
#endif

extern "C" int app_voice_report(APP_STATUS_INDICATION_T status, uint8_t device_id)
{
    return app_voice_report_handler(status, device_id, true);
}

extern "C" int app_voice_report_generic(APP_STATUS_INDICATION_T status, uint8_t device_id, uint8_t isMerging)
{
    return app_voice_report_handler(status, device_id, isMerging);
}

#endif

#define POWERON_PRESSMAXTIME_THRESHOLD_MS (5000)

static osThreadId apps_init_tid = NULL;
static enum APP_POWERON_CASE_T pwron_case = APP_POWERON_CASE_INVALID;
void apps_set_init_tid_signal(int32_t signal)
{
    osSignalSet(apps_init_tid, signal);
}

void apps_set_power_on_case(uint32_t flag)
{
    pwron_case = (APP_POWERON_CASE_T)flag;
}

void app_poweron_wait_finished(void)
{
    osSignalWait(0x2, osWaitForever);
}

#ifndef APP_TEST_MODE
static uint8_t app_poweron_wait_case(void)
{
    uint32_t stime, etime;

#ifdef __POWERKEY_CTRL_ONOFF_ONLY__
    pwron_case = APP_POWERON_CASE_NORMAL;
#else
    pwron_case = APP_POWERON_CASE_INVALID;

    stime = hal_sys_timer_get();
    osSignalWait(0x2, POWERON_PRESSMAXTIME_THRESHOLD_MS);
    etime = hal_sys_timer_get();
    LOG_DEBUG("powon raw case:%d time:%d", pwron_case, TICKS_TO_MS(etime - stime));
#endif
    return pwron_case;
}
#endif

extern "C" int system_shutdown(void);

int app_shutdown(void)
{
    system_shutdown();
    return 0;
}

int system_reset(void);
int app_reset(void)
{
    system_reset();
    return 0;
}

extern "C" void app_notify_stack_ready()
{
    LOG_DEBUG("app_notify_stack_ready");
    osSignalSet(apps_init_tid, 0x3);
}

static void app_wait_stack_ready(void)
{
    uint32_t stime, etime;
    stime = hal_sys_timer_get();
    osSignalWait(0x3, 1000);
    etime = hal_sys_timer_get();
    LOG_DEBUG("app_wait_stack_ready: wait:%d ms", TICKS_TO_MS(etime - stime));
}

static void app_postponed_reset_timer_handler(void const *param);
osTimerDef(APP_POSTPONED_RESET_TIMER, app_postponed_reset_timer_handler);
static osTimerId app_postponed_reset_timer = NULL;
#define APP_RESET_PONTPONED_TIME_IN_MS 2000
static void app_postponed_reset_timer_handler(void const *param)
{
    hal_sw_bootmode_set(HAL_SW_BOOTMODE_ENTER_HIDE_BOOT);
    system_reset();
}

void app_start_postponed_reset(void)
{
    if (NULL == app_postponed_reset_timer)
    {
        app_postponed_reset_timer = osTimerCreate(osTimer(APP_POSTPONED_RESET_TIMER), osTimerOnce, NULL);
    }

    osTimerStart(app_postponed_reset_timer, APP_RESET_PONTPONED_TIME_IN_MS);
}

#ifdef __BTIF_AUTOPOWEROFF__
#ifdef __TWS__
extern tws_dev_t tws;
#endif

void CloseEarphone(void)
{
    int activeCons;
    osapi_lock_stack();
    activeCons = btif_me_get_activeCons();
    osapi_unlock_stack();

#ifdef ANC_APP
    if (app_anc_work_status())
    {
        app_poweroff_timer.timer_en     = 1;
        app_poweroff_timer.timer_period = 30;
        return;
    }
#endif
#ifdef __TWS__
    if (activeCons == 0 || ((tws.tws_source.connected == false) &&
                            (app_tws_get_conn_state() == TWS_MASTER_CONN_SLAVE)))
#else
    if (activeCons == 0)
#endif
    {
        LOG_DEBUG("!!!CloseEarphone\n");
        app_shutdown();
    }
}
#endif /* __AUTOPOWEROFF__ */

void    a2dp_suspend_music_force(void);
uint8_t app_poweroff_flag = 0;

extern uint8_t     app_tws_auto_poweroff;
extern "C" uint8_t is_sco_mode(void);
static bool app_in_charging_mode = false;

bool app_is_power_off_in_progress(void)
{
    return app_poweroff_flag?TRUE:FALSE;
}

int app_deinit(int deinit_case)
{
    int nRet = 0;
    LOG_DEBUG("%s case:%d", __func__, deinit_case);

#ifdef IS_MULTI_AI_ENABLED
    ai_manager_info_save_before_reboot();
#endif

#ifdef __PC_UART_MOD_CUSTOM__
    app_uart_close();
#endif

#if defined(BTUSB_AUDIO_MODE)
    if (app_usbaudio_mode_on())
        return 0;
#endif

    app_battery_close();
    if(app_in_charging_mode)
        return 0;

    if (!deinit_case)
    {
        app_poweroff_flag = 1;
        app_status_indication_filter_set(APP_STATUS_INDICATION_BOTHSCAN);
        app_status_indication_set(APP_STATUS_INDICATION_POWEROFF);

        if (is_sco_mode())
        {
            app_bt_disconnect_sco_link();
            osDelay(200);
        }

        a2dp_suspend_music_force();
        osDelay(200);
#ifndef FPGA
        nv_record_flash_flush();
#endif

        dump_whole_logs_in_normal_case();

        if (btif_me_get_activeCons())
        {    
            app_start_custom_function_in_bt_thread(0,
                                               0,
                                               ( uint32_t )conn_bt_disconnect_all);
            osDelay(3000);
        }

#ifdef IAG_BLE_INCLUDE
        //actively disconnect ble link
        app_start_custom_function_in_bt_thread(0,
                                               0,
                                               ( uint32_t )appm_disconnect_all);
#endif

#ifdef MEDIA_PLAYER_SUPPORT
        app_voice_report(APP_STATUS_INDICATION_POWEROFF, 0);
#endif
        osDelay(2000);
        af_close();
        app_poweroff_flag = 0;
    }

    return nRet;
}

#ifdef APP_TEST_MODE
extern void app_test_init(void);
int         app_init(void)
{
    int     nRet       = 0;
    uint8_t pwron_case = APP_POWERON_CASE_INVALID;
    LOG_DEBUG("%s", __func__);
    app_poweroff_flag = 0;

    apps_init_tid = osThreadGetId();
    app_sysfreq_req(APP_SYSFREQ_USER_APP_0, APP_SYSFREQ_52M);
    list_init();
    af_open();
    app_os_init();
    app_pwl_open();
    app_audio_open();
    app_audio_manager_open();
    app_overlay_open();
    if (app_key_open(true))
    {
        nRet = -1;
        goto exit;
    }

    app_test_init();
exit:
    app_sysfreq_req(APP_SYSFREQ_USER_APP_0, APP_SYSFREQ_32K);
    return nRet;
}
#else
#define NVRAM_ENV_FACTORY_TESTER_STATUS_TEST_PASS (0xffffaa55)
int app_bt_connect2tester_init(void)
{
    btif_device_record_t rec;
    bt_bdaddr_t tester_addr;
    uint8_t i;
    bool find_tester = false;
    struct nvrecord_env_t *nvrecord_env;
    btdevice_profile *btdevice_plf_p;
    uint8_t *btAddr = factory_section_get_bt_address();
    nv_record_env_get(&nvrecord_env);

    if (nvrecord_env->factory_tester_status.status != NVRAM_ENV_FACTORY_TESTER_STATUS_DEFAULT)
        return 0;

    if (btAddr)
    {
        memcpy(tester_addr.address, btAddr, BTIF_BD_ADDR_SIZE);
        nv_record_open(section_usrdata_ddbrecord);
        for (i = 0; nv_record_enum_dev_records(i, &rec) == BT_STS_SUCCESS; i++)
        {
            if (!memcmp(rec.bdAddr.address, tester_addr.address, BTIF_BD_ADDR_SIZE))
            {
                find_tester = true;
            }
        }
        if (i == 0 && !find_tester)
        {
            memset(&rec, 0, sizeof(btif_device_record_t));
            memcpy(rec.bdAddr.address, tester_addr.address, BTIF_BD_ADDR_SIZE);
            nv_record_add(section_usrdata_ddbrecord, &rec);
            btdevice_plf_p = ( btdevice_profile * )app_bt_profile_active_store_ptr_get(rec.bdAddr.address);
            btdevice_plf_p->hfp_act = true;
            btdevice_plf_p->a2dp_act = true;
        }
        if (find_tester && i > 2)
        {
            nv_record_ddbrec_delete(&tester_addr);
            nvrecord_env->factory_tester_status.status = NVRAM_ENV_FACTORY_TESTER_STATUS_TEST_PASS;
            nv_record_env_set(nvrecord_env);
        }
    }

    return 0;
}

int app_nvrecord_rebuild(void)
{
    struct nvrecord_env_t *nvrecord_env;
    nv_record_env_get(&nvrecord_env);

    nv_record_sector_clear();
    nv_record_env_init();
    uint32_t lk = nv_record_pre_write_operation();
    nvrecord_env->factory_tester_status.status = NVRAM_ENV_FACTORY_TESTER_STATUS_TEST_PASS;
    nv_record_post_write_operation(lk);
    nv_record_env_set(nvrecord_env);
    nv_record_extension_clear();
    nv_record_flash_flush();
    return 0;
}

#ifdef __TWS__
extern void app_tws_start_reconnct(struct tws_mode_t *tws_mode);
extern uint16_t app_tws_delay_count;
#endif

void bt_change_to_jlink(APP_KEY_STATUS *status, void *param);
void bt_enable_tports(void);

extern void app_tws_start_pairing_in_chargerbox();

void app_ble_tws_initialized_callback_handler(void)
{
    // TODO: add the handlings executed when the BLE tws initialized is done

#if BLE_TWS_USE_BLE_AS_INBOX_COMMUNICATION_PATH
    if (app_tws_ui_is_in_repairing())
    {
        app_tws_start_pairing_in_chargerbox();
    }
#endif
}

void app_tws_config_bt_address(void)
{
    APP_TWS_PRINT_MODE();

    if (app_tws_is_master_mode())
    {
        memcpy(bt_addr, tws_mode_ptr()->masterAddr.address, 6);
    }
    else if (app_tws_is_slave_mode())
    {
        memcpy(bt_addr, tws_mode_ptr()->slaveAddr.address, 6);
    }
    else if (app_tws_is_freeman_mode())
    {
        //do noting
    }
    else
    {
        ASSERT(0, "INVALID TWS MODE:%d", app_tws_get_mode());
    }

    LOG_DEBUG("%s TWS address is:", __func__);
    DUMP8("0x%02x ", bt_addr, 6);
}

static void app_config_ble_tws(void)
{
    BLE_TWS_CONFIG_T tBleTwsConfig;
    tBleTwsConfig.earSide = app_tws_get_earside();
    tBleTwsConfig.earSideAsMasterByDefault = TWS_RIGHT_SIDE;
    tBleTwsConfig.rssiThreshold = TWS_BOX_RSSI_THRESHOLD;
    tBleTwsConfig.ble_tws_initialized_cb = app_ble_tws_initialized_callback_handler;

    ble_tws_config(&tBleTwsConfig);
}

#if OS_HAS_CPU_STAT
extern "C" void rtx_show_all_threads_usage(void);
#define CPU_USAGE_TIMER_TMO_VALUE (_CPU_STATISTICS_PEROID_ / 3)
static void cpu_usage_timer_handler(void const *param);
osTimerDef(cpu_usage_timer, cpu_usage_timer_handler);
static osTimerId cpu_usage_timer_id = NULL;
static void cpu_usage_timer_handler(void const *param)
{
    rtx_show_all_threads_usage();
}
#endif
#ifdef __TWS_PAIR_DIRECTLY__
extern "C" void app_bt_start_pairing(uint32_t pairingMode, uint8_t *pMasterAddr, uint8_t *SlaveAddr);
void app_tws_searching_slave();
#endif

//#define FORCE_SIGNALINGMODE
//#define FORCE_NOSIGNALINGMODE
//#define FORCE_LBRT_NOSIGNALINGMODE

#if _GFPS_
static void app_tell_battery_info_handler(uint8_t *batteryValueCount,
                                          uint8_t *batteryValue)
{
    GFPS_BATTERY_STATUS_E status;
    if (app_battery_is_charging())
    {
        status = BATTERY_CHARGING;
    }
    else
    {
        status = BATTERY_NOT_CHARGING;
    }

    // TODO: add the charger case's battery level
    if (IS_CONNECTED_WITH_TWS())
    {
        *batteryValueCount = 2;
    }
    else
    {
        *batteryValueCount = 1;
    }

    if (1 == *batteryValueCount)
    {
        batteryValue[0] = (app_battery_current_level() * 10) | (status << 7);
    }
    else
    {
        // if (app_tws_is_left_side())
        // {
        //     batteryValue[0] = (app_battery_current_level() * 10) | (status << 7);
        //     batteryValue[1] = (app_tws_get_peer_device_battery_level() * 10) | (status << 7);
        // }
        // else
        {
            // batteryValue[0] = (app_tws_get_peer_device_battery_level() * 10) | (status << 7);
            batteryValue[0] = (app_battery_current_level() * 10) | (status << 7);
            batteryValue[1] = (app_battery_current_level() * 10) | (status << 7);
        }
    }
}
#endif

extern "C" void gsound_dump_set_flag(uint8_t is_happend);
int app_init(void)
{
    int nRet = 0;
    struct nvrecord_env_t *nvrecord_env;
    bool need_check_key = true;
    bool isInCharging = false;
    uint8_t pwron_case = APP_POWERON_CASE_INVALID;
    LOG_DEBUG("app_init");
    nv_record_init();
#ifdef __WATCHER_DOG_RESET__
    app_wdt_open(15);
#endif
#ifdef ANC_APP
    app_anc_ios_init();
#endif
    app_sysfreq_req(APP_SYSFREQ_USER_APP_0, APP_SYSFREQ_104M);
#if defined(MCU_HIGH_PERFORMANCE_MODE)
    LOG_DEBUG("sys freq calc : %d", hal_sys_timer_calc_cpu_freq(100, 0));
#endif
    apps_init_tid = osThreadGetId();
    list_init();
    app_os_init();
    app_status_indication_init();

#ifdef BTADDR_FOR_DEBUG
    gen_bt_addr_for_debug();
#endif

#ifdef FORCE_SIGNALINGMODE
    hal_sw_bootmode_clear(HAL_SW_BOOTMODE_TEST_BRIDGEMODE | HAL_SW_BOOTMODE_TEST_LBRT_BRIDGEMODE);
    hal_sw_bootmode_set(HAL_SW_BOOTMODE_TEST_MODE | HAL_SW_BOOTMODE_TEST_SIGNALINGMODE);
#elif defined FORCE_NOSIGNALINGMODE
    hal_sw_bootmode_clear(HAL_SW_BOOTMODE_TEST_SIGNALINGMODE | HAL_SW_BOOTMODE_TEST_LBRT_BRIDGEMODE);
    hal_sw_bootmode_set(HAL_SW_BOOTMODE_TEST_MODE | HAL_SW_BOOTMODE_TEST_BRIDGEMODE);
#elif defined FORCE_LBRT_NOSIGNALINGMODE
    hal_sw_bootmode_clear(HAL_SW_BOOTMODE_TEST_BRIDGEMODE | HAL_SW_BOOTMODE_TEST_SIGNALINGMODE);
    hal_sw_bootmode_set(HAL_SW_BOOTMODE_TEST_MODE | HAL_SW_BOOTMODE_TEST_LBRT_BRIDGEMODE);
#endif

#if defined(DUMP_CRASH_ENABLE)
    if (hal_sw_bootmode_get() & HAL_SW_BOOTMODE_REBOOT_FROM_CRASH)
    {
        hal_sw_bootmode_clear(HAL_SW_BOOTMODE_REBOOT_FROM_CRASH);
        LOG_DEBUG("Crash happened!!!");
        gsound_dump_set_flag(true);
    }
#endif



#ifdef __PC_UART_MOD_CUSTOM__
    app_uart_open();
#endif

#if OS_HAS_CPU_STAT
    cpu_usage_timer_id = osTimerCreate(osTimer(cpu_usage_timer), osTimerPeriodic, NULL);
    if (cpu_usage_timer_id != NULL)
    {
        osTimerStart(cpu_usage_timer_id, CPU_USAGE_TIMER_TMO_VALUE);
    }
#endif
    app_bt_init();

    if (hal_sw_bootmode_get() & HAL_SW_BOOTMODE_REBOOT)
    {
        hal_sw_bootmode_clear(HAL_SW_BOOTMODE_REBOOT);
        pwron_case = APP_POWERON_CASE_REBOOT;
        need_check_key = false;
        LOG_DEBUG("REBOOT!!!");
    }

    if (hal_sw_bootmode_get() & HAL_SW_BOOTMODE_TEST_MODE)
    {
        hal_sw_bootmode_clear(HAL_SW_BOOTMODE_TEST_MODE);
        pwron_case = APP_POWERON_CASE_TEST;
        need_check_key = false;
        LOG_DEBUG("TESTER!!!");
    }

    nRet = app_battery_open();
    LOG_DEBUG("%d", nRet);
#if defined(__CHARGING_POWERON_AS_NORMAL__)
    nRet = 0;
    need_check_key = false;
    pwron_case = APP_POWERON_CASE_NORMAL;
#else
    if (nRet < 0)
    {
        nRet = -1;
        goto exit;
    }
    else if (nRet > 0 &&
             pwron_case != APP_POWERON_CASE_TEST)
    {
        pwron_case = APP_POWERON_CASE_CHARGING;
        app_in_charging_mode = true;
        app_status_indication_set(APP_STATUS_INDICATION_CHARGING);
        LOG_DEBUG("CHARGING!!!");
        app_battery_start();
#if !defined(BTUSB_AUDIO_MODE)
        btdrv_start_bt();
        btdrv_hciopen();
        btdrv_sleep_config(1);
        btdrv_hcioff();
#endif
        app_key_open(false);
        app_key_init_on_charging();
        nRet = 0;
        goto exit;
    }
    else
    {
        nRet = 0;
    }
#endif

    hal_sw_bootmode_set(HAL_SW_BOOTMODE_REBOOT);
#if defined(_AUTO_TEST_)
    AUTO_TEST_SEND("Power on.");
#endif

#if defined(__FORCE_OTABOOT_UPDATE__)
    apps_ota_checker();
#endif

#ifdef __TWS_RECONNECT_USE_BLE__
    app_tws_ble_reconnect_init();
    app_tws_delay_count = 400;
#endif

    af_open();
    app_audio_open();
    app_audio_manager_open();
    app_overlay_open();

    nv_record_env_init();
    factory_section_open();
    app_config_ble_tws();
    nv_record_env_update_ble_addr();
    app_refresh_random_seed();

// #ifdef __TWS_PAIR_DIRECTLY__
//     app_tws_config_bt_mode_after_btroleswitch();
// #endif
    app_tws_config_bt_address();
    // app_bt_connect2tester_init();
    nv_record_env_get(&nvrecord_env);

#ifdef IS_MULTI_AI_ENABLED
    init_ai_mode();
#endif

#ifdef __AI_VOICE__
    if (ai_open())
    {
        nRet = -1;
        goto exit;
    }
#ifdef KNOWLES_UART_DATA
        app_ai_voice_uart_audio_init();
#endif  
#endif

#if defined(HALFDUPLEXUART)
    usr_halfduplexuart_open(115200);
#endif
#ifdef AUDIO_DEBUG_V0_1_0
    speech_tuning_init();
#endif
#ifdef ANC_APP
    app_anc_open_module();
#endif

#ifdef MEDIA_PLAYER_SUPPORT
    app_play_audio_set_lang(nvrecord_env->media_language.language);
#endif

    app_bt_stream_volume_ptr_update(NULL);

#ifdef __THIRDPARTY
    app_thirdparty_specific_lib_event_handle(THIRDPARTY_ID_NO2,THIRDPARTY_INIT);
#endif

    btdrv_start_bt();

    if (pwron_case != APP_POWERON_CASE_TEST)
    {
        BesbtInit();
        app_wait_stack_ready();
        bt_drv_extra_config_after_init();

#ifdef __TWS__
        app_ble_add_tws_dev_to_whitelist();
#endif

#if _GFPS_
        app_fp_rfcomm_init();
        app_gfps_set_battery_info_acquire_handler(app_tell_battery_info_handler);
#endif
    }

    if (app_key_open(need_check_key))
    {
        nRet = -1;
        goto exit;
    }

    app_sysfreq_req(APP_SYSFREQ_USER_APP_0, APP_SYSFREQ_52M);

    LOG_DEBUG("bt_stack_init_done:%d", pwron_case);

    if (pwron_case == APP_POWERON_CASE_REBOOT)
    {
        app_start_custom_function_in_bt_thread(( uint32_t )1,
                                               0,
                                               ( uint32_t )btif_me_write_bt_inquiry_scan_type);

        app_start_custom_function_in_bt_thread(( uint32_t )1,
                                               0,
                                               ( uint32_t )btif_me_write_bt_page_scan_type);
        app_start_custom_function_in_bt_thread(( uint32_t )1,
                                               0,
                                               ( uint32_t )btif_me_write_bt_sleep_enable);

        btdrv_set_lpo_times();

        app_tws_ui_init();

        ble_tws_set_bt_ready_flag();

#ifdef __AI_VOICE__
        app_ai_spp_server_init();
#endif

#if GSOUND_OTA_ENABLED && VOICE_DATAPATH
        GSoundOtaHandlerInit();
#endif
#ifdef BES_OTA_BASIC
        bes_ota_init();
#endif

        app_bt_accessmode_set(BTIF_BAM_NOT_ACCESSIBLE);
        app_key_init();
        app_battery_start();
        app_start_auto_power_off_supervising();

        if (!isInCharging)
        {
            conn_system_boot_up_handler();
        }

#if defined(IAG_BLE_INCLUDE) && defined(BTIF_BLE_APP_DATAPATH_SERVER)
        BLE_custom_command_init();
#endif
#ifdef __THIRDPARTY
        app_thirdparty_specific_lib_event_handle(THIRDPARTY_ID_NO1,THIRDPARTY_START);
        app_thirdparty_specific_lib_event_handle(THIRDPARTY_ID_NO2,THIRDPARTY_BT_CONNECTABLE);
#endif
    }
#ifdef __ENGINEER_MODE_SUPPORT__
    else if (pwron_case == APP_POWERON_CASE_TEST)
    {
        app_factorymode_set(true);
        app_status_indication_set(APP_STATUS_INDICATION_POWERON);

#ifdef MEDIA_PLAYER_SUPPORT
        app_voice_report(APP_STATUS_INDICATION_POWERON, 0);
#endif

#ifdef __WATCHER_DOG_RESET__
        app_wdt_close();
#endif
        LOG_DEBUG("!!!!!ENGINEER_MODE!!!!!");
        nRet = 0;
        app_nvrecord_rebuild();
        if (hal_sw_bootmode_get() & HAL_SW_BOOTMODE_TEST_SIGNALINGMODE)
        {
            hal_sw_bootmode_clear(HAL_SW_BOOTMODE_TEST_MASK);
            app_factorymode_bt_signalingtest(NULL, NULL);
        }
        if (hal_sw_bootmode_get() & HAL_SW_BOOTMODE_TEST_BRIDGEMODE)
        {
            hal_sw_bootmode_clear(HAL_SW_BOOTMODE_TEST_MASK);
            app_factorymode_bt_bridgemode(NULL, NULL);
        }
        if (hal_sw_bootmode_get() & HAL_SW_BOOTMODE_TEST_LBRT_BRIDGEMODE)
        {
            hal_sw_bootmode_clear(HAL_SW_BOOTMODE_TEST_MASK);
            app_factorymode_lbrt_bridgemode(NULL, NULL);
        }
        app_factorymode_key_init();
    }
#endif
    else
    {
        app_status_indication_set(APP_STATUS_INDICATION_POWERON);

#ifdef MEDIA_PLAYER_SUPPORT
        app_voice_report(APP_STATUS_INDICATION_POWERON, 0);
#endif

#if defined(IAG_BLE_INCLUDE) && defined(BTIF_BLE_APP_DATAPATH_SERVER)
        BLE_custom_command_init();
#endif

#if !defined(__CHARGING_POWERON_AS_NORMAL__) && !defined(NO_PWRKEY)
        if (pwron_case != APP_POWERON_CASE_CHARGING)
        {
            app_poweron_key_init();
            pwron_case = app_poweron_wait_case();
        }
#else
#if defined(NO_PWRKEY) && defined(__TWS_PAIR_DIRECTLY__)
        if (app_tws_is_freeman_mode())
            pwron_case = APP_POWERON_CASE_BOTHSCAN;
        else
#endif
            pwron_case = APP_POWERON_CASE_NORMAL;
#endif
        {
            if (pwron_case != APP_POWERON_CASE_INVALID && pwron_case != APP_POWERON_CASE_DITHERING)
            {
                LOG_DEBUG("Power on case:%d", pwron_case);
                nRet = 0;
#ifndef __POWERKEY_CTRL_ONOFF_ONLY__
                app_status_indication_set(APP_STATUS_INDICATION_INITIAL);
#endif
                app_start_custom_function_in_bt_thread(( uint32_t )1,
                                                       0,
                                                       ( uint32_t )btif_me_write_bt_inquiry_scan_type);

                app_start_custom_function_in_bt_thread(( uint32_t )1,
                                                       0,
                                                       ( uint32_t )btif_me_write_bt_page_scan_type);

                app_start_custom_function_in_bt_thread(( uint32_t )1,
                                                       0,
                                                       ( uint32_t )btif_me_write_bt_sleep_enable);

                btdrv_set_lpo_times();

                app_tws_ui_init();

                ble_tws_set_bt_ready_flag();

#ifdef __AI_VOICE__
                if (ai_open())
                {
                    nRet = -1;
                    goto exit;
                }
                app_ai_spp_server_init();
#endif

#if GSOUND_OTA_ENABLED && VOICE_DATAPATH
                GSoundOtaHandlerInit();
#endif
#ifdef BES_OTA_BASIC
                bes_ota_init();
#endif

#ifdef __INTERCONNECTION__
                app_interconnection_init();
#endif

                switch (pwron_case)
                {
                case APP_POWERON_CASE_CALIB:
                    break;

#ifdef __TWS_PAIR_DIRECTLY__
                case APP_POWERON_CASE_BOTHSCAN:
                    LOG_DEBUG("APP_POWERON_CASE_BOTHSCAN !!!");
                    app_bt_start_pairing(0, NULL, NULL);
                    app_start_10_second_timer(APP_PAIR_TIMER_ID);
#if defined(NO_PWRKEY)
                    osDelay(200);
                    app_tws_searching_slave();
#endif
#ifdef __THIRDPARTY
                    app_thirdparty_specific_lib_event_handle(THIRDPARTY_ID_NO2,THIRDPARTY_BT_DISCOVERABLE);
#endif
                    break;
#endif

                case APP_POWERON_CASE_NORMAL:
#if defined(__BTIF_EARPHONE__) && !defined(__EARPHONE_STAY_BOTH_SCAN__)
                    app_bt_accessmode_set(BAM_NOT_ACCESSIBLE);
#endif
                case APP_POWERON_CASE_REBOOT:
                case APP_POWERON_CASE_ALARM:
                default:
                    app_status_indication_set(APP_STATUS_INDICATION_PAGESCAN);
#if defined(__CHARGING_POWERON_AS_NORMAL__)
#if defined(NO_PWRKEY)
                    if (app_tws_is_freeman_mode())
                        app_tws_simulate_pairing();
                    else
                        app_tws_reconnect_after_sys_startup();
#endif
                        //app_tws_freeman_simulate_pairing();
#else
#ifdef __FORCE_BOX_OPEN_FOR_POWER_ON_RECONNECT__
                    //debug bt reconnet
                    app_tws_reconnect_after_sys_startup();
#endif  //__FORCE_BOX_OPEN_FOR_POWER_ON_RECONNECT__
#endif
#ifdef __THIRDPARTY
                    app_thirdparty_specific_lib_event_handle(THIRDPARTY_ID_NO2,THIRDPARTY_BT_CONNECTABLE);
#endif
                    break;
                }
                if (pwron_case != APP_POWERON_CASE_CHARGING)
                {
#if !defined(__POWERKEY_CTRL_ONOFF_ONLY__) && !defined(__CHARGING_POWERON_AS_NORMAL__) && !defined(NO_PWRKEY)
                    app_poweron_wait_finished();
#endif
                }
                app_key_init();
                app_battery_start();

#ifdef __THIRDPARTY
                app_thirdparty_specific_lib_event_handle(THIRDPARTY_ID_NO1, THIRDPARTY_START);
#endif
                app_start_auto_power_off_supervising();
                app_env_checker_timer = osTimerCreate(osTimer(APP_ENV_CHECKER), osTimerPeriodic, NULL);
                osTimerStart(app_env_checker_timer, APP_ENV_CHECKER_INTERVAL_MS);
                app_set_threadhandle(APP_MODUAL_ENVCHECKER, app_env_checker_process);

#if defined(TWS_RBCODEC_PLAYER) || defined(TWS_LINEIN_PLAYER)
                player_role_thread_init();
                rb_ctl_init();
#endif
            }
            else
            {
                af_close();
                app_key_close();
                nRet = -1;
            }
        }
    }
exit:
    // bt_change_to_jlink(NULL,NULL);
    // bt_enable_tports();

#ifdef ANC_APP
    app_anc_set_init_done();
#endif
#if defined(BTUSB_AUDIO_MODE)
    if (pwron_case == APP_POWERON_CASE_CHARGING)
    {

#ifdef __WATCHER_DOG_RESET__
        app_wdt_close();
#endif
        af_open();
        app_key_handle_clear();
        app_usb_key_init();

        app_usbaudio_entry();
    }
#endif

    app_sysfreq_req(APP_SYSFREQ_USER_APP_0, APP_SYSFREQ_32K);
    return nRet;
}
#endif

extern bool app_bt_has_connectivitys(void);

void app_auto_power_off_timer(bool turnon)
{
    //and no active connection
    if (turnon && !app_bt_has_connectivitys())
        app_start_10_second_timer(APP_POWEROFF_TIMER_ID);
    else
        app_stop_10_second_timer(APP_POWEROFF_TIMER_ID);
}

void app_start_auto_power_off_supervising(void)
{
}

void app_stop_auto_power_off_supervising(void)
{
}
