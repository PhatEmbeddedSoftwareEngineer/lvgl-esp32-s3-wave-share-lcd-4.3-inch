#include "lcd.hpp"
ESP_Panel *panel = NULL;

BMS_FRAME       bmsFr;
DCDC_FRAME      dcdcFr;
BATTERY_FRAME   batteryFr;
FAN_FRAME       fanFr;
ACTUATOR_FRAME  actuatorFr;
ADDITION_FRAME  additionFr;

/**Mô phỏng giá trị trên thanh tiến trình */

int DataRandom=0;
int PercentPowerBMS1=0;
int PercentPowerBMS2=0;

/* lớp kế thừa từ lớp con LCD_4_3 */
father _father;

/*tọa độ y của các cell pin */
const int toadoY = -10;

/* for show 14 cell pin  */
static int toadoX14=40;
const int distanceCell14=50;
/**********************************/

/* for show 16 cell pin */

static int toadoX16=-5;
const int distanceCell16=50;

/****************************** */

/* 14 cell pin  */
float cellDji[14];

/* 16 cell pin BMS1*/
float cellBMS1[16];

/*16 cell pin BMS2*/
float cellBMS2[16];

Color color;

// for dji 
lv_obj_t *screenDJI;
lv_obj_t *main_bg;
lv_obj_t *labelWaiting;
lv_obj_t *three_label;
lv_obj_t *labelTension;
lv_obj_t *four_label;
lv_obj_t *labelCurrent;
lv_obj_t *five_label;
lv_obj_t *labelTemperature;
lv_obj_t *six_label;
lv_obj_t *labelChargingTime;
lv_obj_t *version_label;
lv_obj_t *seri_label;

lv_obj_t *label_cellone;
lv_obj_t *label_celltwo;
lv_obj_t *label_cellthree;
lv_obj_t *label_cellfour;
lv_obj_t *label_cellfive;
lv_obj_t *label_cellsix;
lv_obj_t *label_cellseven;
lv_obj_t *label_celleight;
lv_obj_t *label_cellnine;
lv_obj_t *label_cellten;
lv_obj_t *label_celleleven;
lv_obj_t *label_celltwelve;
lv_obj_t *label_cellthirteen;
lv_obj_t *label_cellfourteen;

/* các nhãn hiển thị của BMS1 */
lv_obj_t *screenBMS1;
lv_obj_t *main_bg_BMS1;
lv_obj_t *labelBMS1;
lv_obj_t *three_label_bms1;
lv_obj_t *labelVonBMS1;
lv_obj_t *labelAmpereBMS1;
lv_obj_t *labelTemperatureBMS1;
lv_obj_t *labelChargingTimeBMS1;
lv_obj_t *four_label_bms1;
lv_obj_t *five_label_bms1;
lv_obj_t *six_label_bms1;

lv_obj_t *cellBms1One;
lv_obj_t *cellBms1Two;
lv_obj_t *cellBms1Three;
lv_obj_t *cellBms1Four;
lv_obj_t *cellBms1Five;
lv_obj_t *cellBms1Six;
lv_obj_t *cellBms1Seven;
lv_obj_t *cellBms1Eight;
lv_obj_t *cellBms1Nine;
lv_obj_t *cellBms1Ten;
lv_obj_t *cellBms1Eleven;
lv_obj_t *cellBms1Twelve;
lv_obj_t *cellBms1Thirteen;
lv_obj_t *cellBms1Fourteen;
lv_obj_t *cellBms1Fiveteen;
lv_obj_t *cellBms1Sixteen;

/* các nhãn hiển thị của BMS2*/
lv_obj_t *screenBMS2;
lv_obj_t *main_bg_bms2;
lv_obj_t *labelBMS2;
lv_obj_t *three_label_bms2;
lv_obj_t *labelVonBMS2;
lv_obj_t *labelAmpereBMS2;
lv_obj_t *labelTemperatureBMS2;
lv_obj_t *labelChargingTimeBMS2;
lv_obj_t *four_label_bms2;
lv_obj_t *five_label_bms2;
lv_obj_t *six_label_bms2;

lv_obj_t *cellBms2One;
lv_obj_t *cellBms2Two;
lv_obj_t *cellBms2Three;
lv_obj_t *cellBms2Four;
lv_obj_t *cellBms2Five;
lv_obj_t *cellBms2Six;
lv_obj_t *cellBms2Seven;
lv_obj_t *cellBms2Eight;
lv_obj_t *cellBms2Nine;
lv_obj_t *cellBms2Ten;
lv_obj_t *cellBms2Eleven;
lv_obj_t *cellBms2Twelve;
lv_obj_t *cellBms2Thirteen;
lv_obj_t *cellBms2Fourteen;
lv_obj_t *cellBms2Fiveteen;
lv_obj_t *cellBms2Sixteen;

static void showCellpin(int x, const int distance,showCell _cell,PageType_t type,
                        lv_obj_t *obj);

LCD_4_3::LCD_4_3()
{
    Serial.begin(115200);
};

// CLEAR MÀN HÌNH 
static void lvgl_port_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
    lv_disp_flush_ready(disp);
}

// hàm này dùng đẻ sử dụng chức năng cảm ứng
static void lvgl_port_tp_read(lv_indev_drv_t * indev, lv_indev_data_t * data)
{
    // đọc cảm biến điện dung
    panel->getLcdTouch()->readData();

    bool touched = panel->getLcdTouch()->getTouchState(); // RETURN TRUE OR FALSE CÓ CHẠM HAY KHÔNG 
    // nếu không chạm thì data->state = 0 trạng thái release 
    if(!touched) {
        data->state = LV_INDEV_STATE_REL; 
    } else {
        // nếu chạm thì lấy điểm chạm in ra theo hệ trục x , y

        TouchPoint point = panel->getLcdTouch()->getPoint();
        
        /**
         * cập nhật trạng thái chạm vào data->state 
         */
        data->state = LV_INDEV_STATE_PR;
        /*Set the coordinates*/
        /**
         * cập nhật tọa độ x và y của điểm chạm
         */
        data->point.x = point.x; 
        data->point.y = point.y;

        Serial.printf("Touch point: x %d, y %d\n", point.x, point.y);
    }
}

static void lvgl_port_lock(int timeout_ms)
{
    const TickType_t timeout_ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    xSemaphoreTakeRecursive(gui_mutex, timeout_ticks);
    //xSemaphoreTake(gui_mutex, portMAX_DELAY);
}

static void lvgl_port_unlock(void)
{
    xSemaphoreGiveRecursive(gui_mutex);
}

static void lvgl_port_task(void *arg)
{
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS; // task_delay_ms = 500 
    while (1) {
        if (xSemaphoreTakeRecursive(gui_mutex, portMAX_DELAY) == pdTRUE)
        {
            lvgl_port_lock(-1);
            task_delay_ms = lv_timer_handler(); // update lại task_delay_ms
            lv_task_handler();
            lvgl_port_unlock();
            if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
                task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
            } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
                task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
            }
            xSemaphoreGiveRecursive(gui_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}


static void ClearScreen(void)
{
    lv_obj_clean(lv_scr_act()); 
}

static void InitLCD()
{
    panel = new ESP_Panel();
    lv_init();

    /*khởi tạo buffer cho LVGL*/
    static lv_disp_draw_buf_t draw_buf;
    /* Sử dụng bộ đệm đôi nhanh hơn bộ đệm đơn */

    static uint8_t buf[LVGL_BUF_SIZE*sizeof(lv_color_t)];
    static uint8_t buf2[LVGL_BUF_SIZE*sizeof(lv_color_t)];

    /**
     * kiểm tra xem con trỏ buf có được cấp phát bộ nhớ chưa
     */
    lv_disp_draw_buf_init(&draw_buf, buf, buf2, LVGL_BUF_SIZE);
    static lv_disp_drv_t disp_drv;
    
    lv_disp_drv_init(&disp_drv);
    /**
     * THÌ ESP_PANEL_LCD_H-RES = 800px
     * ESP_PANEL_LCD_V_RES = 480px
     */
    disp_drv.hor_res = ESP_PANEL_LCD_H_RES; // màn hình 800 pixel 
    disp_drv.ver_res = ESP_PANEL_LCD_V_RES; // màn hình 480 pixel 
    /*void (*flush_cb)(struct _lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);*/
    disp_drv.flush_cb = lvgl_port_disp_flush;

    // đóng gói hàm draw_buf và disp_drv 
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /*nếu sử dụng cảm biến chạm thì khởi tạo thêm hàm này*/
#if ESP_PANEL_USE_LCD_TOUCH
#endif

    // khởi tạo bus và device của panel
    panel->init();
    // init i2c port and address 
    ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000);
    expander->init();
    expander->begin();
    expander->multiPinMode(TP_RST | LCD_BL | LCD_RST | SD_CS | USB_SEL, OUTPUT);
    expander->multiDigitalWrite(TP_RST | LCD_BL | LCD_RST | SD_CS, HIGH);
    // Tắt đèn nền
    expander->digitalWrite(USB_SEL, LOW);
    /* Add into panel */
    panel->addIOExpander(expander);
    /* Start panel */
    panel->begin();
    xTaskCreatePinnedToCore(lvgl_port_task,
                            LVGL_TASKNAME,
                            LVGL_TASK_STACK_SIZE,
                            NULL,
                            LVGL_TASK_PRIORITY,
                            &TaskLVGLMAIN_Handler,
                            LVGL_COREID);
    /* Khóa mutex do API LVGL không an toàn cho luồng */
    lvgl_port_lock(-1);
    /* Release the mutex */
    lvgl_port_unlock();
    //free(panel);

}   

/**for pin BMS2 */
static void set_value_BMS2(void *bar,int32_t v)
{
    lv_bar_set_value((lv_obj_t*)bar,PercentPowerBMS2,LV_ANIM_OFF);
}

/**for pin BMS1 */
static void set_value_BMS1(void *bar,int32_t v)
{
    lv_bar_set_value((lv_obj_t*)bar,PercentPowerBMS1,LV_ANIM_OFF);
}

/**for PIN DJI */
static void set_value(void * bar, int32_t v)
{
    lv_bar_set_value((lv_obj_t *)bar, DataRandom, LV_ANIM_OFF);
}
static void event_cb(lv_event_t * e)
{
    /*Lấy mô tả phần vẽ từ sự kiện*/
    lv_obj_draw_part_dsc_t * dsc = lv_event_get_draw_part_dsc(e);
    if(dsc->part != LV_PART_INDICATOR) return;

    lv_obj_t * obj = lv_event_get_target(e);

    /*Khởi tạo cấu trúc mô tả nhãn*/
    
    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.font = LV_FONT_DEFAULT; // font 30 

    /*Lấy giá trị hiện tại của thanh tiến trình và chuyển nó thành chuỗi*/

    char buf[8];
    lv_snprintf(buf, sizeof(buf), "%d", (int)lv_bar_get_value(obj));

    /*Tính kích thước của văn bản*/

    lv_point_t txt_size;
    lv_txt_get_size(&txt_size, buf, label_dsc.font, label_dsc.letter_space, 
    label_dsc.line_space, LV_COORD_MAX,
                    label_dsc.flag);

    /*Xác định khu vực vẽ văn bản*/
    
    lv_area_t txt_area;
    /*If the indicator is long enough put the text inside on the right*/
    if(lv_area_get_width(dsc->draw_area) > txt_size.x + 20) {
        txt_area.x2 = dsc->draw_area->x2 - 5;
        txt_area.x1 = txt_area.x2 - txt_size.x + 1;
        label_dsc.color = lv_color_white();
    }
    /*If the indicator is still short put the text out of it on the right*/
    else {
        txt_area.x1 = dsc->draw_area->x2 + 5;
        txt_area.x2 = txt_area.x1 + txt_size.x - 1;
        label_dsc.color = lv_color_black();
    }

    txt_area.y1 = dsc->draw_area->y1 + (lv_area_get_height(dsc->draw_area) - txt_size.y) / 2;
    txt_area.y2 = txt_area.y1 + txt_size.y - 1;
    
    /*Vẽ nhãn*/
    lv_draw_label(dsc->draw_ctx, &label_dsc, &txt_area, buf, NULL);
}

static void MainBar(Bar type)
{
    //if(additionFr.set==2)
    {
        // Khởi tạo và thiết lập kiểu nền
        static lv_style_t style_bg;
        lv_style_init(&style_bg);
        lv_style_set_bg_color(&style_bg, lv_palette_main(LV_PALETTE_GREY)); 
        lv_style_set_border_width(&style_bg,2);
        lv_style_set_pad_all(&style_bg, 2); /*To make the indicator smaller*/
        lv_style_set_radius(&style_bg, 2);
        lv_style_set_anim_time(&style_bg, 1000);
        lv_style_set_bg_opa(&style_bg, LV_OPA_COVER);

        // Khởi tạo và thiết lập kiểu cho chỉ báo
        static lv_style_t style_indic;
        lv_style_init(&style_indic);
        lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_GREEN)); // Màu xanh lá cây
        lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
        lv_style_set_radius(&style_indic, 2);

        // Tạo thanh tiến trình và áp dụng kiểu
        lv_obj_t * bar = lv_bar_create(lv_scr_act());
        lv_obj_add_style(bar, &style_bg, 0); // Áp dụng kiểu nền cho thanh tiến trình
        lv_obj_add_style(bar, &style_indic, LV_PART_INDICATOR); // Áp dụng kiểu cho chỉ báo

        lv_obj_add_event_cb(bar, event_cb, LV_EVENT_DRAW_PART_END, NULL);
        lv_obj_set_size(bar, 790, 70);
        lv_obj_align(bar, LV_ALIGN_CENTER, 0, -40); // y càng nhỏ thì càng đi lên ngược lại y càng thấp thì càng đi xuống 

        // Tạo và cấu hình hoạt ảnh
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, bar);
        lv_anim_set_values(&a, 0, 100);
        if(type == BARDJI)
            lv_anim_set_exec_cb(&a, set_value);
        else if(type == BARBMS1)
            lv_anim_set_exec_cb(&a, set_value_BMS1);
        else
            lv_anim_set_exec_cb(&a, set_value_BMS2);
        lv_anim_set_time(&a, 100);
        lv_anim_set_playback_time(&a, 2000);
        lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
        lv_anim_start(&a);
        lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);
    }
}
static void clear_ui_Image(void);
static void clear_uiPage2(void)
{
    if(screenBMS1 != NULL)
    {
        lv_obj_del(screenBMS1);
        screenBMS1=NULL;
    }
}
static void clear_uiPage3(void);


static void GUI_Display(PageType_t type)
{
    lv_obj_t **screenAll;
    lv_obj_t **main_bg_All;
    lv_obj_t **labelAll;
    lv_obj_t **labelDienApAll;
    lv_obj_t **labelDataDienAp;
    lv_obj_t **labelDongDien;
    lv_obj_t **labelDataCurrentAll;
    lv_obj_t **labelNhietdo;
    lv_obj_t **labelDataNhietDo;
    lv_obj_t **labelSoLanSac;
    lv_obj_t **labelDataSoLanSac;

    if(type == DJI)
    {
        screenAll = &screenDJI;
        main_bg_All = &main_bg;
        labelAll = &labelWaiting;
        labelDienApAll = &three_label;
        labelDataDienAp = &labelTension;
        labelDongDien = &four_label;
        labelDataCurrentAll = &labelCurrent;
        labelNhietdo = &five_label;
        labelDataNhietDo = &labelTemperature;
        labelSoLanSac = &six_label;
        labelDataSoLanSac = &labelChargingTime;
        clear_uiPage3();
        
        
    }
    else if(type == BMS1)
    {
        screenAll = &screenBMS1;
        main_bg_All = &main_bg_BMS1;
        labelAll = &labelBMS1;
        labelDienApAll = &three_label_bms1;
        labelDataDienAp = &labelVonBMS1;
        labelDongDien = &four_label_bms1;
        labelDataCurrentAll = &labelAmpereBMS1;
        labelNhietdo = &five_label_bms1;
        labelDataNhietDo = &labelTemperatureBMS1;
        labelSoLanSac = &six_label_bms1;
        labelDataSoLanSac = &labelChargingTimeBMS1;
        clear_ui_Image();
        
    }
    else 
    {
        screenAll = &screenBMS2;
        main_bg_All = &main_bg_bms2;
        labelAll = &labelBMS2;
        labelDienApAll = &three_label_bms2;
        labelDataDienAp = &labelVonBMS2;
        labelDongDien = &four_label_bms2;
        labelDataCurrentAll = &labelAmpereBMS2;
        labelNhietdo = &five_label_bms2;
        labelDataNhietDo = &labelTemperatureBMS2;
        labelSoLanSac = &six_label_bms2;
        labelDataSoLanSac = &labelChargingTimeBMS2;
        clear_uiPage2();
        
    }
    *screenAll = lv_obj_create(lv_scr_act());
    lv_obj_set_size(*screenAll,ESP_PANEL_LCD_H_RES,ESP_PANEL_LCD_V_RES);
    lv_obj_center(*screenAll);
    lv_obj_clear_flag(*screenAll, LV_OBJ_FLAG_SCROLLABLE);

    /*hàm này dùng để hiển thị tọa độ của logo TTD*/
    *main_bg_All = lv_img_create(*screenAll);
    lv_img_set_src(*main_bg_All,&logosmall);
    lv_obj_align(*main_bg_All,LV_ALIGN_TOP_LEFT,-20,-30);
    lv_obj_clear_flag(*main_bg_All, LV_OBJ_FLAG_SCROLLABLE);

    *labelAll = lv_label_create(*screenAll);
    lv_obj_set_width(*labelAll, 300);
    if(type == DJI)
    {
        lv_label_set_text(*labelAll, "Đang Đợi");
    }
    else if(type == BMS1)
    {
        lv_label_set_text(*labelAll, "Khối Pin 1");
    }
    else 
        lv_label_set_text(*labelAll, "Khối Pin 2");
    
    lv_obj_align(*labelAll, LV_ALIGN_TOP_MID, 30, 10); // Adjust the y offset as needed
    lv_obj_set_style_text_color(*labelAll, lv_color_hex(color.BLACK), LV_PART_MAIN);
    lv_obj_set_style_text_font(*labelAll, &arial_40, LV_PART_MAIN);
    lv_obj_clear_flag(*labelAll, LV_OBJ_FLAG_SCROLLABLE);

    *labelDienApAll = lv_label_create(*screenAll);
    lv_obj_set_width(*labelDienApAll,300);
    lv_label_set_text(*labelDienApAll,"Điện áp (V)");
    lv_obj_align(*labelDienApAll,LV_ALIGN_LEFT_MID,0,30);
    lv_obj_set_style_text_color(*labelDienApAll,lv_color_hex(color.BLACK),LV_PART_MAIN);
    lv_obj_set_style_text_font(*labelDienApAll,&font_1,LV_PART_MAIN);
    lv_obj_clear_flag(*labelDienApAll, LV_OBJ_FLAG_SCROLLABLE);

    *labelDataDienAp = lv_label_create(*screenAll);
    lv_obj_set_width(*labelDataDienAp, 300);
    lv_label_set_text(*labelDataDienAp, "0.0");
    // lv_label_set_text_fmt(number_label, "Item: %"LV_PRIu32, DataRandom);
    lv_obj_align_to(*labelDataDienAp, *labelDienApAll, LV_ALIGN_OUT_BOTTOM_MID, 10, 10);        // Căn chỉnh phía dưới
    lv_obj_set_style_text_color(*labelDataDienAp, lv_color_hex(color.BLACK), LV_PART_MAIN); // Màu đen
    lv_obj_set_style_text_font(*labelDataDienAp, &arial_40, LV_PART_MAIN);
    lv_obj_clear_flag(*labelDataDienAp, LV_OBJ_FLAG_SCROLLABLE);

    *labelDongDien = lv_label_create(*screenAll);
    lv_obj_set_width(*labelDongDien,300);
    lv_label_set_text(*labelDongDien,"Dòng điện (A)");
    lv_obj_align(*labelDongDien,LV_ALIGN_LEFT_MID,200,30);
    lv_obj_set_style_text_color(*labelDongDien,lv_color_hex(color.BLACK),LV_PART_MAIN);
    lv_obj_set_style_text_font(*labelDongDien,&font_1,LV_PART_MAIN);
    lv_obj_clear_flag(*labelDongDien, LV_OBJ_FLAG_SCROLLABLE);

    *labelDataCurrentAll = lv_label_create(*screenAll);
    lv_obj_set_width(*labelDataCurrentAll, 300);
    lv_label_set_text(*labelDataCurrentAll, "0.0");
    // lv_label_set_text_fmt(number_label, "Item: %"LV_PRIu32, DataRandom);
    lv_obj_align_to(*labelDataCurrentAll, *labelDongDien, LV_ALIGN_OUT_BOTTOM_MID, 10, 10);         // Căn chỉnh phía dưới
    lv_obj_set_style_text_color(*labelDataCurrentAll, lv_color_hex(color.BLACK), LV_PART_MAIN); // Màu đen
    lv_obj_set_style_text_font(*labelDataCurrentAll, &arial_40, LV_PART_MAIN);
    lv_obj_clear_flag(*labelDataCurrentAll, LV_OBJ_FLAG_SCROLLABLE);

    *labelNhietdo = lv_label_create(*screenAll);
    lv_obj_set_width(*labelNhietdo,300);
    lv_label_set_text(*labelNhietdo,"Nhiệt độ (°C)");
    lv_obj_align(*labelNhietdo,LV_ALIGN_LEFT_MID,400,30);
    lv_obj_set_style_text_color(*labelNhietdo,lv_color_hex(color.BLACK),LV_PART_MAIN);
    lv_obj_set_style_text_font(*labelNhietdo,&font_1,LV_PART_MAIN);
    lv_obj_clear_flag(*labelNhietdo, LV_OBJ_FLAG_SCROLLABLE);

    *labelDataNhietDo = lv_label_create(*screenAll);
    lv_obj_set_width(*labelDataNhietDo, 300);
    lv_label_set_text(*labelDataNhietDo, "0.0");
    // lv_label_set_text_fmt(number_label, "Item: %"LV_PRIu32, DataRandom);
    lv_obj_align_to(*labelDataNhietDo, *labelNhietdo, LV_ALIGN_OUT_BOTTOM_MID, 10, 10);         // Căn chỉnh phía dưới
    lv_obj_set_style_text_color(*labelDataNhietDo, lv_color_hex(color.BLACK), LV_PART_MAIN); // Màu đen
    lv_obj_set_style_text_font(*labelDataNhietDo, &arial_40, LV_PART_MAIN);
    lv_obj_clear_flag(*labelDataNhietDo, LV_OBJ_FLAG_SCROLLABLE);

    *labelSoLanSac = lv_label_create(*screenAll);
    lv_obj_set_width(*labelSoLanSac,300);
    lv_label_set_text(*labelSoLanSac,"Số lần sạc");
    lv_obj_align(*labelSoLanSac,LV_ALIGN_LEFT_MID,600,30);
    lv_obj_set_style_text_color(*labelSoLanSac,lv_color_hex(color.BLACK),LV_PART_MAIN);
    lv_obj_set_style_text_font(*labelSoLanSac,&font_1,LV_PART_MAIN);
    lv_obj_clear_flag(*labelSoLanSac, LV_OBJ_FLAG_SCROLLABLE);

    *labelDataSoLanSac = lv_label_create(*screenAll);
    lv_obj_set_width(*labelDataSoLanSac, 300);
    lv_label_set_text(*labelDataSoLanSac, "0");
    // lv_label_set_text_fmt(number_label, "Item: %"LV_PRIu32, DataRandom);
    lv_obj_align_to(*labelDataSoLanSac, *labelSoLanSac, LV_ALIGN_OUT_BOTTOM_MID, 10, 10);          // Căn chỉnh phía dưới
    lv_obj_set_style_text_color(*labelDataSoLanSac, lv_color_hex(color.BLACK), LV_PART_MAIN); // Màu đen
    lv_obj_set_style_text_font(*labelDataSoLanSac, &arial_40, LV_PART_MAIN);
    lv_obj_clear_flag(*labelDataSoLanSac, LV_OBJ_FLAG_SCROLLABLE);

    if(type == DJI)
    {
        if(additionFr.set == CHOOSE::CHOOSE_DJI)
        {
            
            /*hàm này dùng để xác định kích thước của màn hình và chống kéo qua kéo lại*/   
            version_label = lv_label_create(screenDJI);
            lv_obj_set_width(version_label,350);
            lv_label_set_text(version_label,"VER: 3.1.2.19");
            //lv_obj_align_to(version_label,second_label,LV_ALIGN_TOP_RIGHT,0,0);
            lv_obj_align(version_label,LV_ALIGN_TOP_RIGHT,70,0);
            lv_obj_set_style_text_color(version_label,lv_color_hex(color.BLACK),LV_PART_MAIN);
            lv_obj_set_style_text_font(version_label,&lv_font_montserrat_24,LV_PART_MAIN);

            /*
                tạo dòng chữ số seri
            */
            seri_label = lv_label_create(screenDJI);
            lv_obj_set_width(seri_label,350);
            lv_label_set_text(seri_label,"SN: ABCDEFGH123456");
            //lv_obj_align_to(version_label,second_label,LV_ALIGN_TOP_RIGHT,0,0);
            lv_obj_align(seri_label,LV_ALIGN_TOP_RIGHT,70,40);
            lv_obj_set_style_text_color(seri_label,lv_color_hex(color.BLACK),LV_PART_MAIN);
            lv_obj_set_style_text_font(seri_label,&lv_font_montserrat_22,LV_PART_MAIN);
            lv_obj_clear_flag(screenDJI, LV_OBJ_FLAG_SCROLLABLE);

            /*
                hiển thị cell pin 
            */
            showCellpin(toadoX14,distanceCell14,FOURTEEN_CELL,DJI,screenDJI);
            lv_obj_clear_flag(screenDJI, LV_OBJ_FLAG_SCROLLABLE);
        }
        
    }
    else if(type == BMS1)
    {
        if(additionFr.set == CHOOSE::CHOOSE_BMS1)
        {
            showCellpin(toadoX16,distanceCell16,SIXTEEN_CELL,BMS1,screenBMS1);
            lv_obj_clear_flag(screenBMS1, LV_OBJ_FLAG_SCROLLABLE);
        }
        
    }
    else
    {
        if(additionFr.set == CHOOSE::CHOOSE_BMS2)   
        {
            showCellpin(toadoX16,distanceCell16,SIXTEEN_CELL,BMS2,screenBMS2);
            lv_obj_clear_flag(screenBMS2, LV_OBJ_FLAG_SCROLLABLE);
        }
        
    }

}
static void clear_ui_Image(void)
{
    if(screenDJI != NULL)
    {
        lv_obj_del(screenDJI);
        screenDJI = NULL;
    }
}
static void clear_uiPage3(void)
{
    if(screenBMS2 != NULL)
    {
        lv_obj_clean(screenBMS2);
        lv_obj_del(screenBMS2);
        screenBMS2=NULL;
    }
}


static void sangMoLogo();
static void blackBackGround()
{
    lv_obj_t *new_screen = lv_obj_create(NULL); // Tạo một màn hình mới
    lv_obj_set_size(new_screen,ESP_PANEL_LCD_H_RES,ESP_PANEL_LCD_V_RES);
    lv_obj_set_style_bg_color(new_screen, lv_color_hex(0x000000), 0); // Màu trắng
    lv_scr_load(new_screen);                    // Tải màn hình mới
}

void LCD_4_3::stateMachine(int state)
{
    while(1)
    {
        switch(state)
        {
            case START:
            {
                InitLCD();
                delay(1000);
                goto exit;
            }break;
            case SCREEN0:
            {   
                sangMoLogo();
                delay(2000);
                ClearScreen();
                delay(100);
                goto exit;
            }break;
            case SCREEN1:
            {
                GUI_Display(DJI);
                MainBar(BARDJI);
                goto exit;
            }break;
            case SCREEN2:
            {   
                GUI_Display(BMS1);
                MainBar(BARBMS1);
                goto exit;
            }break;
            case SCREEN3:
            {
                GUI_Display(BMS2);
                MainBar(BARBMS2);
                goto exit;
            }break;
            case SCREEN4:
            {
                blackBackGround();
                goto exit;
            }break;
        }
    }
    exit:
        Serial.printf("out\n");
}

void Task_Dummy_Sensor(void *pvParameters)
{
    message_t message;
    while(1)
    {
        if (xSemaphoreTakeRecursive(gui_mutex, portMAX_DELAY) == pdTRUE) 
        {
            memset(&message,0,sizeof(message));
            if(additionFr.set==CHOOSE::CHOOSE_DJI)
            {
                /*For pin DJI*/
                message.dienAp=(double)batteryFr.get.voltage/1000;
                message.currentAmpere=(double)batteryFr.get.current/1000;
                message.Temperature= (double)batteryFr.get.temperature/10;
                message.solanSac=batteryFr.get.numberCharge;
                message.count=batteryFr.get.capacity;
                DataRandom = batteryFr.get.percent;
                /*for 14 cell DJI*/
                if (message.count == BATTERY_UNKNOWN)
                {
                    for(uint8_t i=0;i<14;i++)
                    {
                        message.cellDJI[i] = 0.0;
                        message.seri[i]= batteryFr.get.seriNumber[i];
                    }
                }
                else
                {
                    for(uint8_t i=0;i<14;i++)
                    {
                        message.cellDJI[i]= (float)batteryFr.get.cell[i]/1000;
                        message.seri[i]= batteryFr.get.seriNumber[i];
                    }
                }
                for(uint8_t i=0;i<4;i++)
                {
                    message.version[i] = batteryFr.get.version[i];
                }
            }
            else if(additionFr.set==CHOOSE::CHOOSE_BMS1)
            {
                /*For pin BMS1*/
                /*bar bms1 */
                PercentPowerBMS1 = bmsFr.get[0].packSOC;
                message.dienApBms1 = bmsFr.get[0].packVoltage;
                message.currentAmpereBms1 = (double)bmsFr.get[0].packCurrent;
                message.TemperatureBms1 = (double)bmsFr.get[0].tempAverage;
                message.solanSacBms1 = bmsFr.get[0].bmsCycles;

                /*for 16 cell pin BMS1 and BMS2*/
                for(uint8_t i=0;i<16;i++)
                {
                    message.cellBMS1[i] = bmsFr.get[0].cellVmV[i]/1000.0;
                    //message.cellBMS2[i] = bmsFr.get[1].cellVmV[i];
                }
            }
            else if(additionFr.set==CHOOSE::CHOOSE_BMS2)
            {
                /*For pin BMS2 */
                PercentPowerBMS2 = bmsFr.get[1].packSOC;
                message.dienApBms2 = bmsFr.get[1].packVoltage;
                message.currentAmpereBms2 = (double)bmsFr.get[1].packCurrent;
                message.TemperatureBms2 = (double)bmsFr.get[1].tempAverage;
                message.solanSacBms2 = bmsFr.get[1].bmsCycles;

                /*for 16 cell pin BMS1 and BMS2*/
                for(uint8_t i=0;i<16;i++)
                {
                    //message.cellBMS1[i] = bmsFr.get[0].cellVmV[i];
                    message.cellBMS2[i] = bmsFr.get[1].cellVmV[i]/1000.0;
                }
            }
            int ret = xQueueSend(QueueHandle, (void *)&message, 0);

            if (ret == pdTRUE) {
                // The message was successfully sent.
                //Serial.println("The message was successfully sent.");
            } else if (ret == errQUEUE_FULL) {
                //Serial.println("Task Dummy Sensor was unable to send data into the Queue");
            }  // Queue send check
            xSemaphoreGiveRecursive(gui_mutex);
        }
        
        vTaskDelay((1L * configTICK_RATE_HZ) / 1000L);
    }
}
void choosePinDji(uint16_t choose)
{
    //choose = (choose%6)+1;
    if(choose==BATTERY_UNKNOWN)
        battery = BATTERY_UNKNOWN;
    else if(choose ==BATTERY_T30)
        battery=BATTERY_T30;
    else if(choose == BATTERY_T20P)
        battery = BATTERY_T20P;
    else if(choose == BATTERY_T40)
        battery = BATTERY_T40;
    else if(choose == BATTERY_T25)
        battery = BATTERY_T25;
    else if(choose == BATTERY_T50)
        battery = BATTERY_T50;
}
void pindji(BATTERY_TYPE _battery)
{
    message_t message;
    switch(_battery)
    {
        case BATTERY_UNKNOWN:
        {
            DataRandom = 0;
            lv_label_set_text(labelWaiting, "Đang đợi");
            lv_label_set_text(labelTension, "0.0");
            lv_label_set_text(labelCurrent, "0.0");
            lv_label_set_text(labelTemperature, "0.0");
            lv_label_set_text(labelChargingTime, "0");
        }break;
        case BATTERY_T30:
        {   
            lv_label_set_text(labelWaiting, "T30");
        }break;
        case BATTERY_T20P:
        {
            lv_label_set_text(labelWaiting, "T20P");
        }break;
        case BATTERY_T40:
        {
            lv_label_set_text(labelWaiting, "T40");
        }break;
        case BATTERY_T25:
        {
            lv_label_set_text(labelWaiting, "T25");
        }break;
        case BATTERY_T50:
        {
            lv_label_set_text(labelWaiting, "T50");
        }break;
        default:
            break;
    }
}
static void fixLagLCD()
{
    lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);
}

static void fastConvertDJI(lv_obj_t *a,char buf[],double data)
{
    sprintf(buf,"%.2f",data);
    lv_label_set_text(a,buf);
    fixLagLCD();
}
static void fastConvertForCellDji(lv_obj_t *a, char buf[], float *cellDJI,float *_cell)
{
    sprintf(buf,"%.2f",*cellDJI);
    *_cell = *cellDJI;
    lv_label_set_text(a,buf);
    fixLagLCD();
}

static void updateLabelDji(message_t *data)
{
    if (labelTension != NULL)
    {
        fastConvertDJI(labelTension,data->buf,data->dienAp);
    }

    if (labelCurrent != NULL)
    {
        fastConvertDJI(labelCurrent,data->buf,data->currentAmpere);
    }
    if (labelTemperature != NULL)
    {
        fastConvertDJI(labelTemperature,data->buf,data->Temperature);
    }
    if (labelChargingTime != NULL)
    {
        sprintf(data->buf, "%d", data->solanSac);
        lv_label_set_text(labelChargingTime, data->buf);
        fixLagLCD();
    }
    if (labelWaiting != NULL)
    {
        choosePinDji(data->count);
        pindji(battery);
        fixLagLCD();
    }
    if(version_label != NULL)
    {
        char buf[20];
        sprintf(buf,"vr: %d.%d.%d.%d",data->version[0],data->version[1],data->version[2],data->version[3]);
        lv_label_set_text(version_label,buf);
        fixLagLCD();

    }
    if(seri_label != NULL)
    {
        char buf[19];
        sprintf(buf,"sn: %c%c%c%c%c%c%c%c%c%c%c%c%c%c",data->seri[0],
        data->seri[1],data->seri[2],data->seri[3],data->seri[4],data->seri[5],
        data->seri[6],data->seri[7],data->seri[8],data->seri[9],data->seri[10],
        data->seri[11],data->seri[12],data->seri[13]);
        lv_label_set_text(seri_label,buf);
        fixLagLCD();
    }
    if(label_cellone != NULL)
    {
        fastConvertForCellDji(label_cellone,data->buf,&data->cellDJI[0],&cellDji[0]);
    }
    if(label_celltwo != NULL)
    {
        fastConvertForCellDji(label_celltwo,data->buf,&data->cellDJI[1],&cellDji[1]);
    }
    if(label_cellthree != NULL)
    {
        fastConvertForCellDji(label_cellthree,data->buf,&data->cellDJI[2],&cellDji[2]);
    }
    if(label_cellfour != NULL)
    {
        fastConvertForCellDji(label_cellfour,data->buf,&data->cellDJI[3],&cellDji[3]);
    }
    if(label_cellfive != NULL)
    {
        fastConvertForCellDji(label_cellfive,data->buf,&data->cellDJI[4],&cellDji[4]);
    }
    if(label_cellsix != NULL)
    {
        fastConvertForCellDji(label_cellsix,data->buf,&data->cellDJI[5],&cellDji[5]);
    }
    if(label_cellseven != NULL)
    {
        fastConvertForCellDji(label_cellseven,data->buf,&data->cellDJI[6],&cellDji[6]);
    }
    if(label_celleight != NULL)
    {
        fastConvertForCellDji(label_celleight,data->buf,&data->cellDJI[7],&cellDji[7]);
    }
    if(label_cellnine != NULL)
    {
        fastConvertForCellDji(label_cellnine,data->buf,&data->cellDJI[8],&cellDji[8]);
    }
    if(label_cellten != NULL)
    {
        fastConvertForCellDji(label_cellten,data->buf,&data->cellDJI[9],&cellDji[9]);
    }
    if(label_celleleven != NULL)
    {
        fastConvertForCellDji(label_celleleven,data->buf,&data->cellDJI[10],&cellDji[10]);

    }
    if(label_celltwelve != NULL)
    {
        fastConvertForCellDji(label_celltwelve,data->buf,&data->cellDJI[11],&cellDji[11]);
    }
    if(label_cellthirteen != NULL)
    {
        fastConvertForCellDji(label_cellthirteen,data->buf,&data->cellDJI[12],&cellDji[12]);
    }
    if(label_cellfourteen != NULL)
    {
        fastConvertForCellDji(label_cellfourteen,data->buf,&data->cellDJI[13],&cellDji[13]);
    }
}
static void updateLabelBms1(message_t *data)
{
    if(labelVonBMS1 != NULL)
    {
        fastConvertDJI(labelVonBMS1,data->buf,data->dienApBms1);
    }
    if(labelAmpereBMS1 != NULL)
    {
        fastConvertDJI(labelAmpereBMS1,data->buf,data->currentAmpereBms1);
    }
    if(labelTemperatureBMS1 != NULL)
    {
        fastConvertDJI(labelTemperatureBMS1,data->buf,data->TemperatureBms1);
    }
    if(labelChargingTimeBMS1 != NULL)
    {
        sprintf(data->buf, "%d", data->solanSacBms1);
        lv_label_set_text(labelChargingTimeBMS1, data->buf);
        fixLagLCD();
    }
    if(cellBms1One != NULL)
    {
        fastConvertForCellDji(cellBms1One,data->buf,&data->cellBMS1[0],&cellBMS1[0]);
    }
    if(cellBms1Two != NULL)
    {
        fastConvertForCellDji(cellBms1Two,data->buf,&data->cellBMS1[1],&cellBMS1[1]);
    }
    if(cellBms1Three != NULL)
    {
        fastConvertForCellDji(cellBms1Three,data->buf,&data->cellBMS1[2],&cellBMS1[2]);
    }
    if(cellBms1Four != NULL)
    {
        fastConvertForCellDji(cellBms1Four,data->buf,&data->cellBMS1[3],&cellBMS1[3]);
    }
    if(cellBms1Five != NULL)
    {
        fastConvertForCellDji(cellBms1Five,data->buf,&data->cellBMS1[4],&cellBMS1[4]);
    }
    if(cellBms1Six != NULL)
    {
        fastConvertForCellDji(cellBms1Six,data->buf,&data->cellBMS1[5],&cellBMS1[5]);
    }
    if(cellBms1Seven != NULL)
    {
        fastConvertForCellDji(cellBms1Seven,data->buf,&data->cellBMS1[6],&cellBMS1[6]);
    }
    if(cellBms1Eight != NULL)
    {
        fastConvertForCellDji(cellBms1Eight,data->buf,&data->cellBMS1[7],&cellBMS1[7]);
    }
    if(cellBms1Nine != NULL)
    {
        fastConvertForCellDji(cellBms1Nine,data->buf,&data->cellBMS1[8],&cellBMS1[8]);
    }
    if(cellBms1Ten != NULL)
    {
        fastConvertForCellDji(cellBms1Ten,data->buf,&data->cellBMS1[9],&cellBMS1[9]);
    }
    if(cellBms1Eleven != NULL)
    {
        fastConvertForCellDji(cellBms1Eleven,data->buf,&data->cellBMS1[10],&cellBMS1[10]);
    }
    if(cellBms1Twelve != NULL)
    {
        fastConvertForCellDji(cellBms1Twelve,data->buf,&data->cellBMS1[11],&cellBMS1[11]);
    }
    if(cellBms1Thirteen != NULL)
    {
        fastConvertForCellDji(cellBms1Thirteen,data->buf,&data->cellBMS1[12],&cellBMS1[12]);
    }
    if(cellBms1Fourteen != NULL)
    {
        fastConvertForCellDji(cellBms1Fourteen,data->buf,&data->cellBMS1[13],&cellBMS1[13]);
    }
    if(cellBms1Fiveteen != NULL)
    {
        fastConvertForCellDji(cellBms1Fiveteen,data->buf,&data->cellBMS1[14],&cellBMS1[14]);
    }
    if(cellBms1Sixteen != NULL)
    {
        fastConvertForCellDji(cellBms1Sixteen,data->buf,&data->cellBMS1[15],&cellBMS1[15]);
    }
}

static void updateLabelBms2(message_t *data)
{
    if(labelVonBMS2 != NULL)
    {
        fastConvertDJI(labelVonBMS2,data->buf,data->dienApBms2);
    }
    if(labelAmpereBMS2 != NULL)
    {
        fastConvertDJI(labelAmpereBMS2,data->buf,data->currentAmpereBms2);
    }
    if(labelTemperatureBMS2 != NULL)
    {
        fastConvertDJI(labelTemperatureBMS2,data->buf,data->TemperatureBms2);
    }
    if(labelChargingTimeBMS2 != NULL)
    {
        sprintf(data->buf, "%d", data->solanSacBms2);
        lv_label_set_text(labelChargingTimeBMS2, data->buf);
        fixLagLCD();
    }
    if(cellBms2One != NULL)
    {
        fastConvertForCellDji(cellBms2One,data->buf,&data->cellBMS2[0],&cellBMS2[0]);
    }
    if(cellBms2Two != NULL)
    {
        fastConvertForCellDji(cellBms2Two,data->buf,&data->cellBMS2[1],&cellBMS2[1]);
    }
    if(cellBms2Three != NULL)
    {
        fastConvertForCellDji(cellBms2Three,data->buf,&data->cellBMS2[2],&cellBMS2[2]);
    }
    if(cellBms2Four != NULL)
    {
        fastConvertForCellDji(cellBms2Four,data->buf,&data->cellBMS2[3],&cellBMS2[3]);
    }
    if(cellBms2Five != NULL)
    {
        fastConvertForCellDji(cellBms2Five,data->buf,&data->cellBMS2[4],&cellBMS2[4]);
    }
    if(cellBms2Six != NULL)
    {
        fastConvertForCellDji(cellBms2Six,data->buf,&data->cellBMS2[5],&cellBMS2[5]);
    }
    if(cellBms2Seven != NULL)
    {
        fastConvertForCellDji(cellBms2Seven,data->buf,&data->cellBMS2[6],&cellBMS2[6]);
    }
    if(cellBms2Eight != NULL)
    {
        fastConvertForCellDji(cellBms2Eight,data->buf,&data->cellBMS2[7],&cellBMS2[7]);
    }
                        
    if(cellBms2Nine != NULL)
    {
        fastConvertForCellDji(cellBms2Nine,data->buf,&data->cellBMS2[8],&cellBMS2[8]);
    }
    if(cellBms2Ten != NULL)
    {
        fastConvertForCellDji(cellBms2Ten,data->buf,&data->cellBMS2[9],&cellBMS2[9]);
    }
    if(cellBms2Eleven != NULL)
    {
        fastConvertForCellDji(cellBms2Eleven,data->buf,&data->cellBMS2[10],&cellBMS2[10]);
    }
    if(cellBms2Twelve != NULL)
    {
        fastConvertForCellDji(cellBms2Twelve,data->buf,&data->cellBMS2[11],&cellBMS2[11]);
    }
    if(cellBms2Thirteen != NULL)
    {
        fastConvertForCellDji(cellBms2Thirteen,data->buf,&data->cellBMS2[12],&cellBMS2[12]);
    }
    if(cellBms2Fourteen != NULL)
    {
        fastConvertForCellDji(cellBms2Fourteen,data->buf,&data->cellBMS2[13],&cellBMS2[13]);
    }
    if(cellBms2Fiveteen != NULL)
    {
        fastConvertForCellDji(cellBms2Fiveteen,data->buf,&data->cellBMS2[14],&cellBMS2[14]);
    }
    if(cellBms2Sixteen != NULL)
    {
        fastConvertForCellDji(cellBms2Sixteen,data->buf,&data->cellBMS2[15],&cellBMS2[15]);
    }
}
static void Task_Screen_Update(void *pvParameters)
{
    message_t message;
    while(1)
    {
        //Serial.printf("\n[Task_Screen_Update] running on core: %d, Free stack space: %d\n", xPortGetCoreID(), uxTaskGetStackHighWaterMark(NULL));
        if (QueueHandle != NULL) 
        {
            if (xQueueReceive(QueueHandle, &message, portMAX_DELAY))
            {
                if (xSemaphoreTakeRecursive(gui_mutex, portMAX_DELAY) == pdTRUE)
                {
                    if (additionFr.set==CHOOSE::CHOOSE_DJI)
                    {
                        updateLabelDji(&message);
                    }
                    // for BMS1
                    else if(additionFr.set==CHOOSE::CHOOSE_BMS1)
                    {
                        updateLabelBms1(&message);
                    }
                    else if(additionFr.set==CHOOSE::CHOOSE_BMS2)
                    {
                        updateLabelBms2(&message);
                    }
                    // giải phóng hàng đợi
                    xSemaphoreGiveRecursive(gui_mutex);
                }
            }
        } 
    }
}
/*các biến dùng để chuyển màn với fix lỗi chống đơ dữ liệu hiển thị*/
static bool page2=true;
static bool page3=true;
static bool page4=true;
static bool page5=true;
static void updateLCDScreen(void *pvParameters)
{
    //static lv_obj_t imgFanZero;
    while(1)
    {
        if (xSemaphoreTakeRecursive(gui_mutex, portMAX_DELAY) == pdTRUE)
        {
            if(additionFr.set==CHOOSE::CHOOSE_DJI)
            {
                if(page2)
                {
                    _father.stateMachine(additionFr.set);
                    page2=false;
                }
                page4=true;
            }
            else if(additionFr.set==CHOOSE::CHOOSE_BMS1)
            {
                if(page3)
                {
                    _father.stateMachine(additionFr.set);
                    page3=false;
                }
            }
            else if(additionFr.set==CHOOSE::CHOOSE_BMS2)
            {
                if(page4)
                {
                    _father.stateMachine(additionFr.set);
                    page4=false;
                }
                page2=true;
                page3=true;
            }
            else if(additionFr.set==5)
            {
                if(page5)
                {
                    _father.stateMachine(additionFr.set);
                    page5=false;
                }
            }
            else
            {
                _father.stateMachine(additionFr.set);
            }
            
            xSemaphoreGiveRecursive(gui_mutex);
        }
        
        //Serial.printf("skipScreen:= %d\n",additionFr.set);
        vTaskDelay(pdMS_TO_TICKS(10));
        
    }
}
void LCD_4_3::initMuititask(void)
{
    xTaskCreatePinnedToCore(Task_Dummy_Sensor,
                            GETDATA_TASKNAME,
                            GETDATA_STACKSIZE,
                            NULL,
                            GETDATA_PRIORITY,
                            &TaskGetData_Handler,
                            GETDATA_COREID);
    xTaskCreatePinnedToCore(Task_Screen_Update,
                            UPDATE_TASKNAME,
                            UPDATE_STACKSIZE,
                            NULL,
                            UPDATE_PRIORITY, 
                            &TaskUpdataScreen_Handler,
                            UPDATE_COREID); // core 1
    xTaskCreatePinnedToCore(updateLCDScreen,
                            CHANGE_SCREEN,
                            CHANGE_SCREEN_STACKSIZE,
                            NULL,
                            CHANGE_SCREEN_PRIORITY,
                            &TaskUpdateLCD_Handler,
                            CHANGE_SCREEN_COREID);

}

static float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
    // Tính toán tỷ lệ ánh xạ
    float fromRange = fromHigh - fromLow;
    float toRange = toHigh - toLow;
    float scale = toRange / fromRange;
    
    // Ánh xạ giá trị từ phạm vi đầu vào sang phạm vi đầu ra
    return toLow + ((value - fromLow) * scale);
}

static void fourteenCellDJi(float cellDJI,int *data)
{
    *data = (int)mapFloat(cellDJI, 3.1, 4.25, 0, 40);
}


/*for bms2 */

typedef void (*fptrBms2)(void *bar, int32_t temp);

static void dataCellBMS2one(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[0],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
static void dataCellBMS2two(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[1],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
static void dataCellBMS2three(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[2],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
static void dataCellBMS2four(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[3],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

static void dataCellBMS2five(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[4],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
static void dataCellBMS2six(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[5],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
static void dataCellBMS2seven(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[6],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
static void dataCellBMS2eight(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[7],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

static void dataCellBMS2nine(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[8],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
static void dataCellBMS2ten(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[9],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
static void dataCellBMS2eleven(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[10],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
static void dataCellBMS2twelve(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[11],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

static void dataCellBMS2thirteen(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[12],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
static void dataCellBMS2fourteen(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[13],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
static void dataCellBMS2fifteen(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[14],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
static void dataCellBMS2sixteen(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[15],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

fptrBms2 taskBms2[]={
    dataCellBMS2one,
    dataCellBMS2two,
    dataCellBMS2three,
    dataCellBMS2four,
    dataCellBMS2five,
    dataCellBMS2six,
    dataCellBMS2seven,
    dataCellBMS2eight,
    dataCellBMS2nine,
    dataCellBMS2ten,
    dataCellBMS2eleven,
    dataCellBMS2twelve,
    dataCellBMS2thirteen,
    dataCellBMS2fourteen,
    dataCellBMS2fifteen,
    dataCellBMS2sixteen
};


/*for bms1 */
typedef void (*fptrBms1)(void *bar, int32_t temp);

static void dataCellBMS1one(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[0],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
static void dataCellBMS1two(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[1],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
static void dataCellBMS1three(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[2],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
static void dataCellBMS1four(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[3],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
static void dataCellBMS1five(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[4],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
static void dataCellBMS1six(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[5],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
static void dataCellBMS1seven(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[6],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
static void dataCellBMS1eight(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[7],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
static void dataCellBMS1nine(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[8],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
static void dataCellBMS1ten(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[9],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
static void dataCellBMS1eleven(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[10],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
static void dataCellBMS1twelve(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[11],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
static void dataCellBMS1thirteen(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[12],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
static void dataCellBMS1fourteen(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[13],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
static void dataCellBMS1fifteen(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[14],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
static void dataCellBMS1sixteen(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[15],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}


fptrBms1 taskBms1[]={
    dataCellBMS1one,
    dataCellBMS1two,
    dataCellBMS1three,
    dataCellBMS1four,
    dataCellBMS1five,
    dataCellBMS1six,
    dataCellBMS1seven,
    dataCellBMS1eight,
    dataCellBMS1nine,
    dataCellBMS1ten,
    dataCellBMS1eleven,
    dataCellBMS1twelve,
    dataCellBMS1thirteen,
    dataCellBMS1fourteen,
    dataCellBMS1fifteen,
    dataCellBMS1sixteen
};

typedef void (*fptrDji)(void *bar, int32_t temp);

static void dataCellOne(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[0],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}

static void dataCellTwo(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[1],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

static void dataCellThree(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[2],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

static void dataCellFour(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[3],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

static void dataCellFive(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[4],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

static void dataCellSix(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[5],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

static void dataCellSeven(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[6],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

static void dataCellEight(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[7],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

static void dataCellnine(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[8],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

static void dataCellTen(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[9],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

static void dataCellEleven(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[10],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

static void dataCellTwelve(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[11],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

static void dataCellThirteen(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[12],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}

static void dataCellFourteen(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[13],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}

fptrDji taskDji[]={
    dataCellOne,
    dataCellTwo,
    dataCellThree,
    dataCellFour,
    dataCellFive,
    dataCellSix,
    dataCellSeven,
    dataCellEight,
    dataCellnine,
    dataCellTen,
    dataCellEleven,
    dataCellTwelve,
    dataCellThirteen,
    dataCellFourteen,
};

static void GUI_DISPLAY_BARSMALL(PageType_t type,uint8_t numberCell,lv_obj_t *parent,lv_obj_t *barCell,int32_t label_x,int32_t label_y,lv_anim_t *hoatAnh)
{
    lv_obj_t **cellAllOne;
    lv_obj_t **cellAllTwo;
    lv_obj_t **cellAllThree;
    lv_obj_t **cellAllFour;
    lv_obj_t **cellAllFive;
    lv_obj_t **cellAllSix;
    lv_obj_t **cellAllSeven;
    lv_obj_t **cellAllEight;
    lv_obj_t **cellAllNine;
    lv_obj_t **cellAllTen;
    lv_obj_t **cellAllEleven;
    lv_obj_t **cellAllTwelve;
    lv_obj_t **cellAllThirteen;
    lv_obj_t **cellAlllFourteen;
    lv_obj_t **cellALLFifTteen;
    lv_obj_t **cellAllSixteen;
    if(type == DJI)
    {
        cellAllOne = &label_cellone;
        cellAllTwo = &label_celltwo;
        cellAllThree = &label_cellthree;
        cellAllFour = &label_cellfour;
        cellAllFive = &label_cellfive;
        cellAllSix = &label_cellsix;
        cellAllSeven = &label_cellseven;
        cellAllEight = &label_celleight;
        cellAllNine = &label_cellnine;
        cellAllTen = &label_cellten;
        cellAllEleven = &label_celleleven;
        cellAllTwelve = &label_celltwelve;
        cellAllThirteen = &label_cellthirteen;
        cellAlllFourteen = &label_cellfourteen;
    }   
    else if(type == BMS1)
    {
        cellAllOne = &cellBms1One;
        cellAllTwo = &cellBms1Two;
        cellAllThree = &cellBms1Three;
        cellAllFour = &cellBms1Four;
        cellAllFive = &cellBms1Five;
        cellAllSix = &cellBms1Six;
        cellAllSeven = &cellBms1Seven;
        cellAllEight = &cellBms1Eight;
        cellAllNine = &cellBms1Nine;
        cellAllTen = &cellBms1Ten;
        cellAllEleven = &cellBms1Eleven;
        cellAllTwelve = &cellBms1Twelve;
        cellAllThirteen = &cellBms1Thirteen;
        cellAlllFourteen = &cellBms1Fourteen;
        cellALLFifTteen = &cellBms1Fiveteen;
        cellAllSixteen = &cellBms1Sixteen;
    }
    else
    {
        cellAllOne = &cellBms2One;
        cellAllTwo = &cellBms2Two;
        cellAllThree = &cellBms2Three;
        cellAllFour = &cellBms2Four;
        cellAllFive = &cellBms2Five;
        cellAllSix = &cellBms2Six;
        cellAllSeven = &cellBms2Seven;
        cellAllEight = &cellBms2Eight;
        cellAllNine = &cellBms2Nine;
        cellAllTen = &cellBms2Ten;
        cellAllEleven = &cellBms2Eleven;
        cellAllTwelve = &cellBms2Twelve;
        cellAllThirteen = &cellBms2Thirteen;
        cellAlllFourteen = &cellBms2Fourteen;
        cellALLFifTteen = &cellBms2Fiveteen;
        cellAllSixteen = &cellBms2Sixteen;
    }
    if(numberCell == 1)
    {
        // Tạo nhãn hiển thị số dưới thanh bar
        *cellAllOne = lv_label_create(parent);
        lv_label_set_text(*cellAllOne, "4.25"); // Đặt số ban đầu
        lv_obj_align_to(*cellAllOne, barCell, LV_ALIGN_OUT_BOTTOM_MID, label_x, label_y); // Căn chỉnh nhãn phía dưới thanh bar
        lv_obj_set_style_text_color(*cellAllOne, lv_color_hex(0x000000), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(*cellAllOne, &lv_font_montserrat_18, LV_PART_MAIN);

        if(type == DJI)
            lv_anim_set_exec_cb(hoatAnh,taskDji[numberCell-1]);
        else if(type == BMS1)
            lv_anim_set_exec_cb(hoatAnh,taskBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,taskBms2[numberCell-1]);
    }
    if(numberCell == 2)
    {
        *cellAllTwo = lv_label_create(parent);
        lv_label_set_text(*cellAllTwo, "4.25"); // Đặt số ban đầu
        lv_obj_align_to(*cellAllTwo, barCell, LV_ALIGN_OUT_BOTTOM_MID, label_x, label_y); // Căn chỉnh nhãn phía dưới thanh bar
        lv_obj_set_style_text_color(*cellAllTwo, lv_color_hex(0x000000), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(*cellAllTwo, &lv_font_montserrat_18, LV_PART_MAIN);

        //lv_anim_init(hoatAnh);
        if(type == DJI)
            lv_anim_set_exec_cb(hoatAnh,taskDji[numberCell-1]);
        else if(type == BMS1)
            lv_anim_set_exec_cb(hoatAnh,taskBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,taskBms2[numberCell-1]);
    }
    if(numberCell == 3)
    {
        // Tạo nhãn hiển thị số dưới thanh bar
        *cellAllThree = lv_label_create(parent);
        lv_label_set_text(*cellAllThree, "4.25"); // Đặt số ban đầu
        lv_obj_align_to(*cellAllThree, barCell, LV_ALIGN_OUT_BOTTOM_MID, label_x, label_y); // Căn chỉnh nhãn phía dưới thanh bar
        lv_obj_set_style_text_color(*cellAllThree, lv_color_hex(0x000000), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(*cellAllThree, &lv_font_montserrat_18, LV_PART_MAIN);

        if(type == DJI)
            lv_anim_set_exec_cb(hoatAnh,taskDji[numberCell-1]);
        else if(type == BMS1)
            lv_anim_set_exec_cb(hoatAnh,taskBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,taskBms2[numberCell-1]);
    }
    if(numberCell == 4)
    {
        *cellAllFour = lv_label_create(parent);
        lv_label_set_text(*cellAllFour, "4.25"); // Đặt số ban đầu
        lv_obj_align_to(*cellAllFour, barCell, LV_ALIGN_OUT_BOTTOM_MID, label_x, label_y); // Căn chỉnh nhãn phía dưới thanh bar
        lv_obj_set_style_text_color(*cellAllFour, lv_color_hex(0x000000), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(*cellAllFour, &lv_font_montserrat_18, LV_PART_MAIN);

        //lv_anim_init(hoatAnh);
        if(type == DJI)
            lv_anim_set_exec_cb(hoatAnh,taskDji[numberCell-1]);
        else if(type == BMS1)
            lv_anim_set_exec_cb(hoatAnh,taskBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,taskBms2[numberCell-1]);
    }
    if(numberCell == 5)
    {
        *cellAllFive = lv_label_create(parent);
        lv_label_set_text(*cellAllFive, "4.25"); // Đặt số ban đầu
        lv_obj_align_to(*cellAllFive, barCell, LV_ALIGN_OUT_BOTTOM_MID, label_x, label_y); // Căn chỉnh nhãn phía dưới thanh bar
        lv_obj_set_style_text_color(*cellAllFive, lv_color_hex(0x000000), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(*cellAllFive, &lv_font_montserrat_18, LV_PART_MAIN);

        //lv_anim_init(hoatAnh);
        if(type == DJI)
            lv_anim_set_exec_cb(hoatAnh,taskDji[numberCell-1]);
        else if(type == BMS1)
            lv_anim_set_exec_cb(hoatAnh,taskBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,taskBms2[numberCell-1]);
    }
    if(numberCell == 6)
    {
        *cellAllSix = lv_label_create(parent);
        lv_label_set_text(*cellAllSix, "4.25"); // Đặt số ban đầu
        lv_obj_align_to(*cellAllSix, barCell, LV_ALIGN_OUT_BOTTOM_MID, label_x, label_y); // Căn chỉnh nhãn phía dưới thanh bar
        lv_obj_set_style_text_color(*cellAllSix, lv_color_hex(0x000000), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(*cellAllSix, &lv_font_montserrat_18, LV_PART_MAIN);

        //lv_anim_init(hoatAnh);
        if(type == DJI)
            lv_anim_set_exec_cb(hoatAnh,taskDji[numberCell-1]);
        else if(type == BMS1)
            lv_anim_set_exec_cb(hoatAnh,taskBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,taskBms2[numberCell-1]);
    }
    if(numberCell == 7)
    {
        // Tạo nhãn hiển thị số dưới thanh bar
        *cellAllSeven = lv_label_create(parent);
        lv_label_set_text(*cellAllSeven, "4.25"); // Đặt số ban đầu
        lv_obj_align_to(*cellAllSeven, barCell, LV_ALIGN_OUT_BOTTOM_MID, label_x, label_y); // Căn chỉnh nhãn phía dưới thanh bar
        lv_obj_set_style_text_color(*cellAllSeven, lv_color_hex(0x000000), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(*cellAllSeven, &lv_font_montserrat_18, LV_PART_MAIN);

        if(type == DJI)
            lv_anim_set_exec_cb(hoatAnh,taskDji[numberCell-1]);
        else if(type == BMS1)
            lv_anim_set_exec_cb(hoatAnh,taskBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,taskBms2[numberCell-1]);
    }
    if(numberCell == 8)
    {
        *cellAllEight = lv_label_create(parent);
        lv_label_set_text(*cellAllEight, "4.25"); // Đặt số ban đầu
        lv_obj_align_to(*cellAllEight, barCell, LV_ALIGN_OUT_BOTTOM_MID, label_x, label_y); // Căn chỉnh nhãn phía dưới thanh bar
        lv_obj_set_style_text_color(*cellAllEight, lv_color_hex(0x000000), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(*cellAllEight, &lv_font_montserrat_18, LV_PART_MAIN);

        if(type == DJI)
            lv_anim_set_exec_cb(hoatAnh,taskDji[numberCell-1]);
        else if(type == BMS1)
            lv_anim_set_exec_cb(hoatAnh,taskBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,taskBms2[numberCell-1]);
    }
    if(numberCell == 9)
    {
        // Tạo nhãn hiển thị số dưới thanh bar
        *cellAllNine = lv_label_create(parent);
        lv_label_set_text(*cellAllNine, "4.25"); // Đặt số ban đầu
        lv_obj_align_to(*cellAllNine, barCell, LV_ALIGN_OUT_BOTTOM_MID, label_x, label_y); // Căn chỉnh nhãn phía dưới thanh bar
        lv_obj_set_style_text_color(*cellAllNine, lv_color_hex(0x000000), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(*cellAllNine, &lv_font_montserrat_18, LV_PART_MAIN);

        if(type == DJI)
            lv_anim_set_exec_cb(hoatAnh,taskDji[numberCell-1]);
        else if(type == BMS1)
            lv_anim_set_exec_cb(hoatAnh,taskBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,taskBms2[numberCell-1]);
    }
    if(numberCell == 10)
    {
        *cellAllTen = lv_label_create(parent);
        lv_label_set_text(*cellAllTen, "4.25"); // Đặt số ban đầu
        lv_obj_align_to(*cellAllTen, barCell, LV_ALIGN_OUT_BOTTOM_MID, label_x, label_y); // Căn chỉnh nhãn phía dưới thanh bar
        lv_obj_set_style_text_color(*cellAllTen, lv_color_hex(0x000000), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(*cellAllTen, &lv_font_montserrat_18, LV_PART_MAIN);
        if(type == DJI)
            lv_anim_set_exec_cb(hoatAnh,taskDji[numberCell-1]);
        else if(type == BMS1)
            lv_anim_set_exec_cb(hoatAnh,taskBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,taskBms2[numberCell-1]);
    }
    if(numberCell == 11)
    {
        // Tạo nhãn hiển thị số dưới thanh bar
        *cellAllEleven = lv_label_create(parent);
        lv_label_set_text(*cellAllEleven, "4.25"); // Đặt số ban đầu
        lv_obj_align_to(*cellAllEleven, barCell, LV_ALIGN_OUT_BOTTOM_MID, label_x, label_y); // Căn chỉnh nhãn phía dưới thanh bar
        lv_obj_set_style_text_color(*cellAllEleven, lv_color_hex(0x000000), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(*cellAllEleven, &lv_font_montserrat_18, LV_PART_MAIN);
        if(type == DJI)
            lv_anim_set_exec_cb(hoatAnh,taskDji[numberCell-1]);
        else if(type == BMS1)
            lv_anim_set_exec_cb(hoatAnh,taskBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,taskBms2[numberCell-1]);
    }
    if(numberCell == 12)
    {
        *cellAllTwelve = lv_label_create(parent);
        lv_label_set_text(*cellAllTwelve, "4.25"); // Đặt số ban đầu
        lv_obj_align_to(*cellAllTwelve, barCell, LV_ALIGN_OUT_BOTTOM_MID, label_x, label_y); // Căn chỉnh nhãn phía dưới thanh bar
        lv_obj_set_style_text_color(*cellAllTwelve, lv_color_hex(0x000000), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(*cellAllTwelve, &lv_font_montserrat_18, LV_PART_MAIN);

        if(type == DJI)
            lv_anim_set_exec_cb(hoatAnh,taskDji[numberCell-1]);
        else if(type == BMS1)
            lv_anim_set_exec_cb(hoatAnh,taskBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,taskBms2[numberCell-1]);
    }
    if(numberCell == 13)
    {
        // Tạo nhãn hiển thị số dưới thanh bar
        *cellAllThirteen = lv_label_create(parent);
        lv_label_set_text(*cellAllThirteen, "4.25"); // Đặt số ban đầu
        lv_obj_align_to(*cellAllThirteen, barCell, LV_ALIGN_OUT_BOTTOM_MID, label_x, label_y); // Căn chỉnh nhãn phía dưới thanh bar
        lv_obj_set_style_text_color(*cellAllThirteen, lv_color_hex(0x000000), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(*cellAllThirteen, &lv_font_montserrat_18, LV_PART_MAIN);

        if(type == DJI)
            lv_anim_set_exec_cb(hoatAnh,taskDji[numberCell-1]);
        else if(type == BMS1)
            lv_anim_set_exec_cb(hoatAnh,taskBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,taskBms2[numberCell-1]);
    }
    if(numberCell == 14)
    {
        *cellAlllFourteen = lv_label_create(parent);
        lv_label_set_text(*cellAlllFourteen, "4.25"); // Đặt số ban đầu
        lv_obj_align_to(*cellAlllFourteen, barCell, LV_ALIGN_OUT_BOTTOM_MID, label_x, label_y); // Căn chỉnh nhãn phía dưới thanh bar
        lv_obj_set_style_text_color(*cellAlllFourteen, lv_color_hex(0x000000), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(*cellAlllFourteen, &lv_font_montserrat_18, LV_PART_MAIN);
        if(type == DJI)
            lv_anim_set_exec_cb(hoatAnh,taskDji[numberCell-1]);
        else if(type == BMS1)
            lv_anim_set_exec_cb(hoatAnh,taskBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,taskBms2[numberCell-1]);
    }
    if(type == BMS1 || type == BMS2)
    {
        if(numberCell==15)
        {
            *cellALLFifTteen = lv_label_create(parent);
            lv_label_set_text(*cellALLFifTteen,"4.25");
            lv_obj_align_to(*cellALLFifTteen,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
            lv_obj_set_style_text_color(*cellALLFifTteen,lv_color_hex(0x000000),LV_PART_MAIN);
            lv_obj_set_style_text_font(*cellALLFifTteen,&lv_font_montserrat_18,LV_PART_MAIN);
            if(type == DJI)
                lv_anim_set_exec_cb(hoatAnh,taskDji[numberCell-1]);
            else if(type == BMS1)
                lv_anim_set_exec_cb(hoatAnh,taskBms1[numberCell-1]);
            else 
                lv_anim_set_exec_cb(hoatAnh,taskBms2[numberCell-1]);
        }
        if(numberCell==16)
        {
            *cellAllSixteen = lv_label_create(parent);
            lv_label_set_text(*cellAllSixteen ,"4.25");
            lv_obj_align_to(*cellAllSixteen ,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
            lv_obj_set_style_text_color(*cellAllSixteen ,lv_color_hex(0x000000),LV_PART_MAIN);
            lv_obj_set_style_text_font(*cellAllSixteen ,&lv_font_montserrat_18,LV_PART_MAIN);
            if(type == DJI)
                lv_anim_set_exec_cb(hoatAnh,taskDji[numberCell-1]);
            else if(type == BMS1)
                lv_anim_set_exec_cb(hoatAnh,taskBms1[numberCell-1]);
            else 
                lv_anim_set_exec_cb(hoatAnh,taskBms2[numberCell-1]);
        }
    }
}

// Hàm tiện ích để tạo một thanh bar và nhãn
static void create_bar_with_label(lv_obj_t *parent, lv_style_t *style_bg, lv_style_t *style_indic, int32_t x, int32_t y, int32_t min, 
    int32_t max,int32_t label_x,int32_t label_y, int32_t initial_value, int32_t anim_duration,uint8_t numberCell,PageType_t type)
{
    lv_obj_t *barCell;
    // Tạo thanh bar
    barCell = lv_bar_create(parent);
    lv_obj_add_style(barCell, style_bg, 0);
    lv_obj_add_style(barCell, style_indic, LV_PART_INDICATOR);
    lv_obj_set_size(barCell, 20, 50);
    lv_obj_align(barCell, LV_ALIGN_BOTTOM_LEFT, x, y);
    lv_bar_set_range(barCell, min, max);
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
    lv_anim_t b;
    lv_anim_init(&b);
    if(type==DJI)
    {
        GUI_DISPLAY_BARSMALL(DJI,numberCell,parent,barCell,label_x,label_y,&b);        
    }
    else if(type == BMS1)
    {
        GUI_DISPLAY_BARSMALL(BMS1,numberCell,parent,barCell,label_x,label_y,&b); 
    }
    else if(type == BMS2)
    {
        GUI_DISPLAY_BARSMALL(BMS2,numberCell,parent,barCell,label_x,label_y,&b);  
    }
    lv_anim_set_time(&b, anim_duration);
    lv_anim_set_playback_time(&b, anim_duration);
    lv_anim_set_var(&b, barCell);
    lv_anim_set_values(&b, min, max);
    lv_anim_set_repeat_count(&b, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&b);

    // Đặt giá trị ban đầu cho thanh bar
    lv_bar_set_value(barCell, initial_value, LV_ANIM_OFF);
}

static void showCellpin(int x, const int distance,showCell _cell,PageType_t type,lv_obj_t *obj)
{
    static lv_style_t style_indic;
    lv_style_init(&style_indic);
    lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
    lv_style_set_radius(&style_indic, 2);
    lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_GREEN));
    lv_style_set_bg_grad_color(&style_indic, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_bg_grad_dir(&style_indic, LV_GRAD_DIR_VER);

    static lv_style_t style_bg;
    lv_style_init(&style_bg);
    lv_style_set_border_width(&style_bg, 2);
    lv_style_set_pad_all(&style_bg, 2); /*To make the indicator smaller*/
    lv_style_set_radius(&style_bg, 2);
    lv_style_set_anim_time(&style_bg, 1000);
    lv_style_set_bg_opa(&style_bg, LV_OPA_TRANSP);

    // Tạo thanh bar và nhãn đầu tiên
    for(uint8_t i=0;i<_cell;i++)
    {
            create_bar_with_label(obj, &style_bg, &style_indic, 
                                                        x + i*distance, toadoY, 0, 40, 
                                                        10,0, 0, 100,i+1,type);
    }

}

/* Tạo một hình ảnh và tự động thay đổi màu của nó */
static void update_image_color(lv_obj_t *img, lv_color_t color, lv_opa_t intensity)
{
    lv_obj_set_style_img_recolor_opa(img, intensity, 0);
    lv_obj_set_style_img_recolor(img, color, 0);
}

static void sangMoLogo()
{
     /*KHỞI TẠO STYLE BASE */
    static lv_style_t style_base;
    /*khởi tạo style base */
    lv_style_init(&style_base);
    /*thiết lập độ rộng đường viền là 0 , nghĩa là không có đường viền */
    lv_style_set_border_width(&style_base,0);
    //lv_style_set_bg_img_opa(&style_base,LV_OPA_COVER);

    /*tạo đối tượng màn hình */
    lv_obj_t *screen = lv_obj_create(lv_scr_act());
    /*khởi tạo kích thước pixel của màn hình */
    lv_obj_set_size(screen, ESP_PANEL_LCD_H_RES, ESP_PANEL_LCD_V_RES);
    
    /*khởi tạo hiển thị màn hình ở trung tâm */
    lv_obj_center(screen);
    /*LV_PART_MAIN áp dụng cho phần chính */
    lv_obj_add_style(screen,&style_base,LV_PART_MAIN);
    /*loại bỏ tính năng cuộn của màn hình */
    lv_obj_clear_flag(screen,LV_OBJ_FLAG_SCROLLABLE);

    /* Tạo hình ảnh */
    //LV_IMG_DECLARE(logosmall);
    lv_obj_t *img1 = lv_img_create(screen);
    lv_img_set_src(img1, &logosmall);
    lv_obj_set_size(img1,200,150);
    lv_img_set_zoom(img1, 1024);
    lv_obj_align(img1,LV_ALIGN_CENTER,0,0);

    /* Tạo màu và độ bão hòa cho hình ảnh */
    lv_color_t color = lv_color_hex(0xFFFFFF);  // Chọn màu mới cho hình ảnh
    lv_opa_t intensity = LV_OPA_COVER;   // Chọn độ bão hòa mới cho hình ảnh

    /* Cập nhật màu của hình ảnh */
    update_image_color(img1, color, intensity);
    for(int i=255;i>0;i--)
    {
        intensity=i;
        update_image_color(img1, color, intensity);
        delay(10);
        
    }
}   
