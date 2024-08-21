#include "lcd.hpp"
static ESP_Panel *panel = NULL;

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
// unsigned long lastTickMillis = 0;

/* lớp kế thừa từ lớp con LCD_4_3 */
father _father;
Color color;
/*tọa độ y của các cell pin */
const int toadoY = -40;

/* for show 14 cell pin  */
static int toadoX14=60;
const int distanceCell14=50;
/**********************************/

/* for show 10 cell pin */
static int toadoX10=60;
const int distanceCell10=70;

/***************************** */

/* for show 16 cell pin */

static int toadoX16=20;
const int distanceCell16=50;

/****************************** */

/* 14 cell pin  */
float cellDji[14];

/* 16 cell pin BMS1*/
float cellBMS1[16];

/*16 cell pin BMS2*/
float cellBMS2[16];


/**các nhãn text hiển thị của DJI  */
lv_obj_t *labelCurrent = NULL;
lv_obj_t *labelTension = NULL;
lv_obj_t *labelTemperature = NULL;
lv_obj_t *labelChargingTime = NULL;
lv_obj_t *labelWaiting = NULL;
lv_obj_t *version_label = NULL;
lv_obj_t *seri_label = NULL;

/* các nhãn hiển thị của BMS1 */
lv_obj_t *labelVonBMS1 = NULL;
lv_obj_t *labelAmpereBMS1 = NULL;
lv_obj_t *labelTemperatureBMS1 = NULL;
lv_obj_t *labelChargingTimeBMS1 = NULL;

/* các nhãn hiển thị của BMS2*/

lv_obj_t *labelVonBMS2 = NULL;
lv_obj_t *labelAmpereBMS2 = NULL;
lv_obj_t *labelTemperatureBMS2 = NULL;
lv_obj_t *labelChargingTimeBMS2 = NULL;

/** các nhãn text hiển thị cell pin dji */

lv_obj_t *label_cellone = NULL;
lv_obj_t *label_celltwo = NULL;
lv_obj_t *label_cellthree = NULL;
lv_obj_t *label_cellfour = NULL;
lv_obj_t *label_cellfive = NULL;
lv_obj_t *label_cellsix = NULL;
lv_obj_t *label_cellseven = NULL;
lv_obj_t *label_celleight = NULL;
lv_obj_t *label_cellnine = NULL; 
lv_obj_t *label_cellten = NULL;
lv_obj_t *label_celleleven = NULL;
lv_obj_t *label_celltwelve = NULL;
lv_obj_t *label_cellthirteen = NULL;
lv_obj_t *label_cellfourteen = NULL;


/** các nhãn text hiển thị cell pin bms1 */

lv_obj_t *cellBms1One = NULL;
lv_obj_t *cellBms1Two = NULL;
lv_obj_t *cellBms1Three = NULL;
lv_obj_t *cellBms1Four = NULL;
lv_obj_t *cellBms1Five = NULL;
lv_obj_t *cellBms1Six = NULL;
lv_obj_t *cellBms1Seven = NULL;
lv_obj_t *cellBms1Eight = NULL;
lv_obj_t *cellBms1Nine = NULL;
lv_obj_t *cellBms1Ten = NULL;
lv_obj_t *cellBms1Eleven = NULL;
lv_obj_t *cellBms1Twelve = NULL;
lv_obj_t *cellBms1Thirteen = NULL;
lv_obj_t *cellBms1Fourteen = NULL;
lv_obj_t *cellBms1Fiveteen = NULL;
lv_obj_t *cellBms1Sixteen = NULL;

/** các nhãn text hiển thị cell pin bms2 */

lv_obj_t *cellBms2One = NULL;
lv_obj_t *cellBms2Two = NULL;
lv_obj_t *cellBms2Three = NULL;
lv_obj_t *cellBms2Four = NULL;
lv_obj_t *cellBms2Five = NULL;
lv_obj_t *cellBms2Six = NULL;
lv_obj_t *cellBms2Seven = NULL;
lv_obj_t *cellBms2Eight = NULL;
lv_obj_t *cellBms2Nine = NULL; 
lv_obj_t *cellBms2Ten = NULL;
lv_obj_t *cellBms2Eleven = NULL;
lv_obj_t *cellBms2Twelve = NULL;
lv_obj_t *cellBms2Thirteen = NULL;
lv_obj_t *cellBms2Fourteen = NULL;
lv_obj_t *cellBms2Fiveteen = NULL;
lv_obj_t *cellBms2Sixteen = NULL;

lv_style_t style_indic;
lv_style_t style_bg;
lv_style_t style_bg_MainBar;
lv_style_t style_indic_MainBar;

lv_obj_t *screen1=NULL;
lv_obj_t *screen2=NULL;
lv_obj_t *screen3=NULL;

lv_obj_t *barDJI=NULL;
lv_obj_t *barBMS1=NULL;
lv_obj_t *barBMS2=NULL;

lv_obj_t *barSmallDji=NULL;
lv_obj_t *barSmallBms1=NULL;
lv_obj_t *barSmallBms2=NULL; 

typedef void (*fptr)(void *bar, int32_t temp);

typedef void (*clearStyle)(void);

void clear_style_indic(void)
{
    lv_style_reset(&style_indic);
    lv_style_reset(&style_bg);
}
void clearMainBar(void)
{
    lv_style_reset(&style_bg_MainBar);
    lv_style_reset(&style_indic_MainBar);
}


clearStyle cleanLvStyle[]={
    clear_style_indic,
    clearMainBar,

};


const uint8_t lengCleanLvStyle = sizeof(cleanLvStyle)/sizeof(cleanLvStyle[0]);

void showCellpin(int x, int distance, showCell _cell, PageType_t type);

LCD_4_3::LCD_4_3()
{
    Serial.begin(115200);
};

// CLEAR MÀN HÌNH 
void lvgl_port_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
    lv_disp_flush_ready(disp);
}

// hàm này dùng đẻ sử dụng chức năng cảm ứng
void lvgl_port_tp_read(lv_indev_drv_t * indev, lv_indev_data_t * data)
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
    /**
     * if timeout_ms < 0 thì thời gian là portMAX_DELAY ngược lại là pdMS_TO_TICKS theo timeout_ms 
     */
    const TickType_t timeout_ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    // set up semophore theo timeout_ticks
    xSemaphoreTakeRecursive(gui_mutex, timeout_ticks);
}

/**
 * unclock semaphore 
 */

static void lvgl_port_unlock(void)
{
    xSemaphoreGiveRecursive(gui_mutex);
}

static void lvgl_port_task(void *arg)
{
    

    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS; // task_delay_ms = 500 
    while (1) {
        //Serial.printf("\n[lvgl_port_task] running on core: %d, Free stack space: %d\n", xPortGetCoreID(), uxTaskGetStackHighWaterMark(NULL));

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


void ClearScreen(void)
{
    lv_obj_clean(lv_scr_act()); 
}

void InitLCD()
{
    /*NOTE: Để đăng ký hiển thị cho LVGL, biến lv_disp_draw_buf_t và biến lv_disp_drv_t phải được khởi tạo.
    lv_disp_draw_buf_t chứa (các) bộ đệm đồ họa bên trong được gọi là (các) bộ đệm vẽ.

    lv_disp_drv_t chứa các hàm gọi lại để tương tác với màn hình và thao tác những thứ liên quan đến bản vẽ.

    */
    panel = new ESP_Panel();

    /*KHỞI TẠO LVGL CORE*/
    // STEP1:
    lv_init();
    // STEP2:

    /*khởi tạo buffer cho LVGL*/
    static lv_disp_draw_buf_t draw_buf;
    /* Sử dụng bộ đệm đôi nhanh hơn bộ đệm đơn */
    /* Sử dụng SRAM nội bộ nhanh hơn PSRAM (Lưu ý: 
    Bộ nhớ được phân bổ bằng `malloc` có thể nằm trong PSRAM.) 
    hàm heap_caps_calloc là một con trỏ void nên cần phải ép kiểu về con trỏ (uint8_t *)
    void *heap_caps_calloc(size_t n, size_t size, uint32_t caps);
    */
    static uint8_t buf[LVGL_BUF_SIZE*sizeof(lv_color_t)];
    static uint8_t buf2[LVGL_BUF_SIZE*sizeof(lv_color_t)];

    /**
     * kiểm tra xem con trỏ buf có được cấp phát bộ nhớ chưa
     */
    //assert(buf);
    lv_disp_draw_buf_init(&draw_buf, buf, buf2, LVGL_BUF_SIZE);
    // toàn bộ quá trình trên là khởi tạo buffer cho LVGL
    ///////////////////////////////////////////////////////
    // STEP3:
    /*KHỞI TẠO MÀN HÌNH 
    */
    static lv_disp_drv_t disp_drv;
    
    lv_disp_drv_init(&disp_drv);
    /**THAY ĐỔI ĐỘ PHÂN GIẢI CỦA MÀN HÌNH TẠI ĐÂY
     * VÍ DỤ MÀN HÌNH CÓ Độ phân giải: 800 × 480 px
     * THÌ ESP_PANEL_LCD_H-RES = 800px
     * ESP_PANEL_LCD_V_RES = 480px
     * 
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
    /**
     * Các bo mạch phát triển này yêu cầu sử dụng thiết bị mở rộng IO để định cấu hình màn hình,
     * nên cần phải khởi tạo trước và đăng ký với bảng điều khiển để sử dụng.
     *
     */
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

    /* Tạo tác vụ để chạy tác vụ LVGL theo định kỳ */
    //gui_mutex = xSemaphoreCreateRecursiveMutex();
    //xTaskCreate(lvgl_port_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);
    xTaskCreatePinnedToCore(lvgl_port_task,
                            LVGL_TASKNAME,
                            LVGL_TASK_STACK_SIZE,
                            NULL,
                            LVGL_TASK_PRIORITY,
                            &TaskLVGLMAIN_Handler,
                            LVGL_COREID);
    /* Khóa mutex do API LVGL không an toàn cho luồng */
    lvgl_port_lock(-1);
    // Xóa màn hình bằng cách vẽ lại màn hình trống (hoặc UI mặc định của bạn)
    /////////////
    /* Release the mutex */
    lvgl_port_unlock();


    //free(panel);
}   

/**for pin BMS2 */
typedef void (*fptrBar)(void *bar,int32_t v);

void set_value_BMS2(void *bar,int32_t v)
{
    lv_bar_set_value((lv_obj_t*)bar,PercentPowerBMS2,LV_ANIM_OFF);
}


/**for pin BMS1 */
void set_value_BMS1(void *bar,int32_t v)
{
    lv_bar_set_value((lv_obj_t*)bar,PercentPowerBMS1,LV_ANIM_OFF);
}


/**for PIN DJI */
void set_value(void * bar, int32_t v)
{
    lv_bar_set_value((lv_obj_t *)bar, DataRandom, LV_ANIM_OFF);
}
fptrBar scanBar[]={
    set_value,
    set_value_BMS1,
    set_value_BMS2,
};
const uint8_t lengScanBar = sizeof(scanBar)/sizeof(scanBar[0]);
void event_cb(lv_event_t * e)
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

void MainBar(BAR type)
{
    // Khởi tạo và thiết lập kiểu nền
    for(uint8_t i=0;i<lengCleanLvStyle;i++)
        cleanLvStyle[i];
    lv_style_init(&style_bg_MainBar);
    lv_style_set_bg_color(&style_bg_MainBar, lv_palette_main(LV_PALETTE_GREY)); 
    lv_style_set_border_width(&style_bg_MainBar,2);
    lv_style_set_pad_all(&style_bg_MainBar, 2); /*To make the indicator smaller*/
    lv_style_set_radius(&style_bg_MainBar, 2);
    lv_style_set_anim_time(&style_bg_MainBar, 1000);
    lv_style_set_bg_opa(&style_bg_MainBar, LV_OPA_COVER);

    // Khởi tạo và thiết lập kiểu cho chỉ báo
    
    lv_style_init(&style_indic_MainBar);
    lv_style_set_bg_color(&style_indic_MainBar, lv_palette_main(LV_PALETTE_GREEN)); // Màu xanh lá cây
    lv_style_set_bg_opa(&style_indic_MainBar, LV_OPA_COVER);
    lv_style_set_radius(&style_indic_MainBar, 2);

    // Tạo thanh tiến trình và áp dụng kiểu
    lv_obj_t **barAll;
    if(type==DJIBAR)   barAll=&barDJI;
    else if(type==BMS1BAR) barAll = &barBMS1;
    else barAll = &barBMS2;
    *barAll = lv_bar_create(lv_scr_act());
    lv_obj_add_style(*barAll, &style_bg_MainBar, 0); // Áp dụng kiểu nền cho thanh tiến trình
    lv_obj_add_style(*barAll, &style_indic_MainBar, LV_PART_INDICATOR); // Áp dụng kiểu cho chỉ báo

    lv_obj_add_event_cb(*barAll, event_cb, LV_EVENT_DRAW_PART_END, NULL);
    lv_obj_set_size(*barAll, 790, 70);
    lv_obj_align(*barAll, LV_ALIGN_CENTER, 0, -40); // y càng nhỏ thì càng đi lên ngược lại y càng thấp thì càng đi xuống 

    // Tạo và cấu hình hoạt ảnh
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, *barAll);
    lv_anim_set_values(&a, 0, 100);
    if(type == DJIBAR)
    {
        lv_anim_set_exec_cb(&a, set_value);
    }
    else if(type == BMS1BAR)
    {
        lv_anim_set_exec_cb(&a,set_value_BMS1);
    }
    else if(type == BMS2BAR)
    {
        lv_anim_set_exec_cb(&a,set_value_BMS2);
    }
    
    lv_anim_set_time(&a, 2000);
    lv_anim_set_playback_time(&a, 2000);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);
    lv_obj_clear_flag(*barAll, LV_OBJ_FLAG_SCROLLABLE);
    if(type == DJIBAR)
    {
        // lv_obj_clean(barBMS2);
        // lv_obj_del(barBMS2);
    }
    else if(type == BMS1BAR)
    {
        lv_obj_clean(barDJI);
        lv_obj_del(barDJI);
    }
    else 
    {
        lv_obj_clean(barBMS1);
        lv_obj_del(barBMS1);
    }
    for(uint8_t i=0;i<lengScanBar;i++)
    {
        lv_anim_del(&a,scanBar[i]);
    }
    
    //lv_obj_del(bar);
    //lv_mem_free(bar);
}



void uiPageMain(PAGE type)
{
    /*hàm này dùng để xác định kích thước của màn hình và chống kéo qua kéo lại*/
    lv_obj_t **screenAll;
    if(type == DJIPAGE) screenAll=&screen1;
    if(type == BMS1PAGE) screenAll=&screen2;
    if(type == BMS2PAGE) screenAll=&screen3;
    *screenAll = lv_obj_create(lv_scr_act());
    lv_obj_set_size(*screenAll,ESP_PANEL_LCD_H_RES,ESP_PANEL_LCD_V_RES);
    lv_obj_center(*screenAll);
    lv_obj_clear_flag(*screenAll, LV_OBJ_FLAG_SCROLLABLE);

    /*hàm này dùng để hiển thị tọa độ của logo TTD*/
    //LV_IMG_DECLARE(logosmall);
    lv_obj_t * main_bg = lv_img_create(*screenAll);
    lv_img_set_src(main_bg,&logosmall);
    lv_obj_align(main_bg,LV_ALIGN_TOP_LEFT,-20,-30);
    lv_obj_clear_flag(main_bg, LV_OBJ_FLAG_SCROLLABLE);

    // chữ Điện áp (V)
    //LV_FONT_DECLARE(font_1);
    lv_obj_t  *three_label = lv_label_create(*screenAll);
    lv_obj_set_width(three_label,300);
    lv_label_set_text(three_label,"Điện áp (V)");
    lv_obj_align(three_label,LV_ALIGN_LEFT_MID,0,30);
    lv_obj_set_style_text_color(three_label,lv_color_hex(color.BLACK),LV_PART_MAIN);
    lv_obj_set_style_text_font(three_label,&font_1,LV_PART_MAIN);
    lv_obj_clear_flag(three_label, LV_OBJ_FLAG_SCROLLABLE);

    // chữ "Dòng điện (A)"
    lv_obj_t  *four_label = lv_label_create(*screenAll);
    lv_obj_set_width(four_label,300);
    lv_label_set_text(four_label,"Dòng điện (A)");
    lv_obj_align(four_label,LV_ALIGN_LEFT_MID,200,30);
    lv_obj_set_style_text_color(four_label,lv_color_hex(color.BLACK),LV_PART_MAIN);
    lv_obj_set_style_text_font(four_label,&font_1,LV_PART_MAIN);
    lv_obj_clear_flag(four_label, LV_OBJ_FLAG_SCROLLABLE);

    // chữ "Nhiệt độ (°C)"
    lv_obj_t  *five_label = lv_label_create(*screenAll);
    lv_obj_set_width(five_label,300);
    lv_label_set_text(five_label,"Nhiệt độ (°C)");
    lv_obj_align(five_label,LV_ALIGN_LEFT_MID,400,30);
    lv_obj_set_style_text_color(five_label,lv_color_hex(color.BLACK),LV_PART_MAIN);
    lv_obj_set_style_text_font(five_label,&font_1,LV_PART_MAIN);
    lv_obj_clear_flag(five_label, LV_OBJ_FLAG_SCROLLABLE);

    // chữ "Số lần sạc"
    lv_obj_t  *six_label = lv_label_create(*screenAll);
    lv_obj_set_width(six_label,300);
    lv_label_set_text(six_label,"Số lần sạc");
    lv_obj_align(six_label,LV_ALIGN_LEFT_MID,600,30);
    lv_obj_set_style_text_color(six_label,lv_color_hex(color.BLACK),LV_PART_MAIN);
    lv_obj_set_style_text_font(six_label,&font_1,LV_PART_MAIN);
    lv_obj_clear_flag(six_label, LV_OBJ_FLAG_SCROLLABLE);



    if(type == DJIPAGE)
    {   //lv_obj_t *labelDJI=labelWaiting;
        labelWaiting = lv_label_create(*screenAll);
        lv_obj_set_width(labelWaiting, 300);
        lv_label_set_text(labelWaiting, "Đang Đợi");
        lv_obj_align(labelWaiting, LV_ALIGN_TOP_MID, 30, 10); // Adjust the y offset as needed
        lv_obj_set_style_text_color(labelWaiting, lv_color_hex(color.BLACK), LV_PART_MAIN);
        lv_obj_set_style_text_font(labelWaiting, &arial_40, LV_PART_MAIN);
        lv_obj_clear_flag(labelWaiting, LV_OBJ_FLAG_SCROLLABLE);

        labelTension = lv_label_create(*screenAll);
        lv_obj_set_width(labelTension, 300);
        lv_label_set_text(labelTension, "0.0");
        // lv_label_set_text_fmt(number_label, "Item: %"LV_PRIu32, DataRandom);
        lv_obj_align_to(labelTension, three_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 10);        // Căn chỉnh phía dưới
        lv_obj_set_style_text_color(labelTension, lv_color_hex(color.BLACK), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(labelTension, &arial_40, LV_PART_MAIN);
        lv_obj_clear_flag(labelTension, LV_OBJ_FLAG_SCROLLABLE);

        labelCurrent = lv_label_create(*screenAll);
        lv_obj_set_width(labelCurrent, 300);
        lv_label_set_text(labelCurrent, "0.0");
        // lv_label_set_text_fmt(number_label, "Item: %"LV_PRIu32, DataRandom);
        lv_obj_align_to(labelCurrent, four_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 10);         // Căn chỉnh phía dưới
        lv_obj_set_style_text_color(labelCurrent, lv_color_hex(color.BLACK), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(labelCurrent, &arial_40, LV_PART_MAIN);
        lv_obj_clear_flag(labelCurrent, LV_OBJ_FLAG_SCROLLABLE);
        
        // số dưới chữ "Nhiệt độ (°C)"
        labelTemperature = lv_label_create(*screenAll);
        lv_obj_set_width(labelTemperature, 300);
        lv_label_set_text(labelTemperature, "0.0");
        // lv_label_set_text_fmt(number_label, "Item: %"LV_PRIu32, DataRandom);
        lv_obj_align_to(labelTemperature, five_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 10);         // Căn chỉnh phía dưới
        lv_obj_set_style_text_color(labelTemperature, lv_color_hex(color.BLACK), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(labelTemperature, &arial_40, LV_PART_MAIN);
        lv_obj_clear_flag(labelTemperature, LV_OBJ_FLAG_SCROLLABLE);

        // số dưới dòng chữ "Số lần sạc"
        labelChargingTime = lv_label_create(*screenAll);
        lv_obj_set_width(labelChargingTime, 300);
        lv_label_set_text(labelChargingTime, "0");
        // lv_label_set_text_fmt(number_label, "Item: %"LV_PRIu32, DataRandom);
        lv_obj_align_to(labelChargingTime, six_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 10);          // Căn chỉnh phía dưới
        lv_obj_set_style_text_color(labelChargingTime, lv_color_hex(color.BLACK), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(labelChargingTime, &arial_40, LV_PART_MAIN);
        lv_obj_clear_flag(labelChargingTime, LV_OBJ_FLAG_SCROLLABLE);

        /**
         * tạo dòng chữ phiển bản 
         */
        version_label = lv_label_create(*screenAll);
        lv_obj_set_width(version_label,350);
        lv_label_set_text(version_label,"VER: 3.1.2.19");
        //lv_obj_align_to(version_label,second_label,LV_ALIGN_TOP_RIGHT,0,0);
        lv_obj_align(version_label,LV_ALIGN_TOP_RIGHT,70,0);
        lv_obj_set_style_text_color(version_label,lv_color_hex(color.BLACK),LV_PART_MAIN);
        lv_obj_set_style_text_font(version_label,&lv_font_montserrat_24,LV_PART_MAIN);

        /*
            tạo dòng chữ số seri
        */
        seri_label = lv_label_create(*screenAll);
        lv_obj_set_width(seri_label,350);
        lv_label_set_text(seri_label,"SN: ABCDEFGH123456");
        //lv_obj_align_to(version_label,second_label,LV_ALIGN_TOP_RIGHT,0,0);
        lv_obj_align(seri_label,LV_ALIGN_TOP_RIGHT,70,40);
        lv_obj_set_style_text_color(seri_label,lv_color_hex(color.BLACK),LV_PART_MAIN);
        lv_obj_set_style_text_font(seri_label,&lv_font_montserrat_22,LV_PART_MAIN);
        lv_obj_clear_flag(*screenAll, LV_OBJ_FLAG_SCROLLABLE);

        showCellpin(toadoX14,distanceCell14,FOURTEEN_CELL,DJI);
        lv_obj_clear_flag(*screenAll, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_clean(main_bg);
        lv_obj_clean(three_label);
        lv_obj_clean(four_label);
        lv_obj_clean(five_label);
        lv_obj_clean(six_label);
        lv_obj_clean(labelWaiting);
        lv_obj_clean(labelTension);
        lv_obj_clean(labelCurrent);
        lv_obj_clean(labelTemperature);
        lv_obj_clean(labelChargingTime);
        lv_obj_clean(version_label);
        lv_obj_clean(seri_label);

        //lv_obj_clean(screen3);
        // lv_obj_del(screen3);
        
    }
    else if(type == BMS2PAGE)
    {
        lv_obj_t *labelBMS2;
        //lv_obj_clean(labelBMS2);
        labelBMS2 = lv_label_create(*screenAll);
        lv_obj_set_width(labelBMS2, 300);
        lv_label_set_text(labelBMS2, "Khối Pin 2");
        lv_obj_align(labelBMS2, LV_ALIGN_TOP_MID, 40, 10); // Adjust the y offset as needed
        lv_obj_set_style_text_color(labelBMS2, lv_color_hex(color.BLACK), LV_PART_MAIN);
        lv_obj_set_style_text_font(labelBMS2, &arial_40, LV_PART_MAIN);
        lv_obj_clear_flag(labelBMS2, LV_OBJ_FLAG_SCROLLABLE);

        labelVonBMS2 = lv_label_create(*screenAll);
        lv_obj_set_width(labelVonBMS2, 300);
        lv_label_set_text(labelVonBMS2, "0.0");
        // lv_label_set_text_fmt(number_label, "Item: %"LV_PRIu32, DataRandom);
        lv_obj_align_to(labelVonBMS2, three_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 10);        // Căn chỉnh phía dưới
        lv_obj_set_style_text_color(labelVonBMS2, lv_color_hex(color.BLACK), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(labelVonBMS2, &arial_40, LV_PART_MAIN);
        lv_obj_clear_flag(labelVonBMS2, LV_OBJ_FLAG_SCROLLABLE);

        // số dưới chữ "Dòng điện (A)"
        labelAmpereBMS2 = lv_label_create(*screenAll);
        lv_obj_set_width(labelAmpereBMS2, 300);
        lv_label_set_text(labelAmpereBMS2, "0.0");
        // lv_label_set_text_fmt(number_label, "Item: %"LV_PRIu32, DataRandom);
        lv_obj_align_to(labelAmpereBMS2, four_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 10);         // Căn chỉnh phía dưới
        lv_obj_set_style_text_color(labelAmpereBMS2, lv_color_hex(color.BLACK), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(labelAmpereBMS2, &arial_40, LV_PART_MAIN);
        lv_obj_clear_flag(labelAmpereBMS2, LV_OBJ_FLAG_SCROLLABLE);

        // số dưới chữ "Nhiệt độ (°C)"
        labelTemperatureBMS2 = lv_label_create(*screenAll);
        lv_obj_set_width(labelTemperatureBMS2, 300);
        lv_label_set_text(labelTemperatureBMS2, "0.0");
        // lv_label_set_text_fmt(number_label, "Item: %"LV_PRIu32, DataRandom);
        lv_obj_align_to(labelTemperatureBMS2, five_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 10);         // Căn chỉnh phía dưới
        lv_obj_set_style_text_color(labelTemperatureBMS2, lv_color_hex(color.BLACK), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(labelTemperatureBMS2, &arial_40, LV_PART_MAIN);
        lv_obj_clear_flag(labelTemperatureBMS2, LV_OBJ_FLAG_SCROLLABLE);

        // số dưới dòng chữ "Số lần sạc"
        labelChargingTimeBMS2 = lv_label_create(*screenAll);
        lv_obj_set_width(labelChargingTimeBMS2, 300);
        lv_label_set_text(labelChargingTimeBMS2, "0");
        // lv_label_set_text_fmt(number_label, "Item: %"LV_PRIu32, DataRandom);
        lv_obj_align_to(labelChargingTimeBMS2, six_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 10);          // Căn chỉnh phía dưới
        lv_obj_set_style_text_color(labelChargingTimeBMS2, lv_color_hex(color.BLACK), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(labelChargingTimeBMS2, &arial_40, LV_PART_MAIN);
        lv_obj_clear_flag(labelChargingTimeBMS2, LV_OBJ_FLAG_SCROLLABLE);

        showCellpin(toadoX16,distanceCell16,SIXTEEN_CELL,BMS2);
        lv_obj_clear_flag(*screenAll, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clean(screen2);
        lv_obj_del(screen2);
        
    }
    else if(type == BMS1PAGE)
    {
        lv_obj_t *labelBMS1;
        //lv_obj_clean(labelBMS1);
        labelBMS1 = lv_label_create(*screenAll);
        lv_obj_set_width(labelBMS1, 300);
        lv_label_set_text(labelBMS1, "Khối Pin 1");
        lv_obj_align(labelBMS1, LV_ALIGN_TOP_MID, 40, 10); // Adjust the y offset as needed
        lv_obj_set_style_text_color(labelBMS1, lv_color_hex(color.BLACK), LV_PART_MAIN);
        lv_obj_set_style_text_font(labelBMS1, &arial_40, LV_PART_MAIN);
        lv_obj_clear_flag(labelBMS1, LV_OBJ_FLAG_SCROLLABLE);

        labelVonBMS1 = lv_label_create(*screenAll);
        lv_obj_set_width(labelVonBMS1, 300);
        lv_label_set_text(labelVonBMS1, "0.0");
        // lv_label_set_text_fmt(number_label, "Item: %"LV_PRIu32, DataRandom);
        lv_obj_align_to(labelVonBMS1, three_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 10);        // Căn chỉnh phía dưới
        lv_obj_set_style_text_color(labelVonBMS1, lv_color_hex(color.BLACK), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(labelVonBMS1, &arial_40, LV_PART_MAIN);
        lv_obj_clear_flag(labelVonBMS1, LV_OBJ_FLAG_SCROLLABLE);

        labelAmpereBMS1 = lv_label_create(*screenAll);
        lv_obj_set_width(labelAmpereBMS1, 300);
        lv_label_set_text(labelAmpereBMS1, "0.0");
        // lv_label_set_text_fmt(number_label, "Item: %"LV_PRIu32, DataRandom);
        lv_obj_align_to(labelAmpereBMS1, four_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 10);         // Căn chỉnh phía dưới
        lv_obj_set_style_text_color(labelAmpereBMS1, lv_color_hex(color.BLACK), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(labelAmpereBMS1, &arial_40, LV_PART_MAIN);
        lv_obj_clear_flag(labelAmpereBMS1, LV_OBJ_FLAG_SCROLLABLE);

        labelTemperatureBMS1 = lv_label_create(*screenAll);
        lv_obj_set_width(labelTemperatureBMS1, 300);
        lv_label_set_text(labelTemperatureBMS1, "0.0");
        // lv_label_set_text_fmt(number_label, "Item: %"LV_PRIu32, DataRandom);
        lv_obj_align_to(labelTemperatureBMS1, five_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 10);         // Căn chỉnh phía dưới
        lv_obj_set_style_text_color(labelTemperatureBMS1, lv_color_hex(color.BLACK), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(labelTemperatureBMS1, &arial_40, LV_PART_MAIN);
        lv_obj_clear_flag(labelTemperatureBMS1, LV_OBJ_FLAG_SCROLLABLE);

        labelChargingTimeBMS1 = lv_label_create(*screenAll);
        lv_obj_set_width(labelChargingTimeBMS1, 300);
        lv_label_set_text(labelChargingTimeBMS1, "0");
        // lv_label_set_text_fmt(number_label, "Item: %"LV_PRIu32, DataRandom);
        lv_obj_align_to(labelChargingTimeBMS1, six_label, LV_ALIGN_OUT_BOTTOM_MID, 10, 10);          // Căn chỉnh phía dưới
        lv_obj_set_style_text_color(labelChargingTimeBMS1, lv_color_hex(color.BLACK), LV_PART_MAIN); // Màu đen
        lv_obj_set_style_text_font(labelChargingTimeBMS1, &arial_40, LV_PART_MAIN);
        lv_obj_clear_flag(labelChargingTimeBMS1, LV_OBJ_FLAG_SCROLLABLE);

        showCellpin(toadoX16,distanceCell16,SIXTEEN_CELL,BMS1);
        lv_obj_clear_flag(*screenAll, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clean(screen1);
        lv_obj_del(screen1);
    } 
}   

void blackBackGround()
{
    lv_obj_t *new_screen = lv_obj_create(NULL); // Tạo một màn hình mới
    lv_obj_set_size(new_screen,ESP_PANEL_LCD_H_RES,ESP_PANEL_LCD_V_RES);
    lv_obj_set_style_bg_color(new_screen, lv_color_hex(0x000000), 0); // Màu trắng
    lv_scr_load(new_screen);                    // Tải màn hình mới
}
static void sangMoLogo();
void clearSangMoLogo();
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
                //state = SCREEN0;
                //clearSangMoLogo();
                //goto exit;
            }break;
            case SCREEN0:
            {   
                
                sangMoLogo();
                delay(2000);
                ClearScreen();
                delay(100);
                //state = SCREEN1;
                //goto exit;
            }break;
            case SCREEN1:
            {
                
                //writeLabel("ECOPOWER20");
                /**
                 * mặc đinh là 0
                 * Điện áp : 40 -> 58.80 (double) .2f
                 * Dòng điện: (double) 0->200 .2f
                 * Nhiệt độ: (double) 25->100 .2f 
                 * Số lần sạc: uint16_t 0->2500 
                 * double cell[14];
                 * phiên bản: uint8_t version[4];
                 * Số seri: char array[14];
                 * typedef enum BATTERY_TYPE
                    {
                        BATTERY_UNKNOWN = 0,
                        BATTERY_T30 = 29000,
                        BATTERY_T20P = 13000,
                        BATTERY_T40 = 30000,
                        BATTERY_T25 = 16818,
                        BATTERY_T50 = 33022,
                    }battery_t;
                    battery_t battery;
                    switch(case)

                 */
                //ClearScreen();
                // ui_Image(&labelCurrent,&labelTension,&labelWaiting,&labelTemperature,
                // &labelChargingTime,&version_label,&seri_label);
                uiPageMain(DJIPAGE);
                MainBar(DJIBAR);
                //lv_obj_clean(screen1);
                //
                //state = SCREEN2;
                //goto exit;
            }break;
            case SCREEN2:
            {
                //ClearScreen();
                //lv_obj_clean();
                uiPageMain(BMS1PAGE);
                MainBar(BMS1BAR);
                //goto exit;
            }break;
            case SCREEN3:
            {
                //ClearScreen();
                uiPageMain(BMS2PAGE);
                MainBar(BMS2BAR);
                //goto exit;
            }break;
            case SCREEN4:
            {
                //ClearScreen();
                //RotateImage(STATE_FAN::ACTIVATE);
                blackBackGround();
                //goto exit;
            }break;
            
        }
        break;
    }
    //exit:
        Serial.printf("out\n");
}

static void Task_Dummy_Sensor(void *pvParameters)
{
    message_t message;
    memset(&message,0,sizeof(message));
    //BMS1_t bms1;
    while(1)
    {
        //Serial.printf("\n[Task_Dummy_Sensor] running on core: %d, Free stack space: %d\n", xPortGetCoreID(), uxTaskGetStackHighWaterMark(NULL));
        if (xSemaphoreTakeRecursive(gui_mutex, portMAX_DELAY) == pdTRUE) 
        {
            if(additionFr.set==2)
            {
                /*For pin DJI*/
                //Serial.printf("batteryFr.get.voltage:= %d\n",batteryFr.get.voltage/1000);
                message.dienAp=(double)batteryFr.get.voltage/1000;
                message.currentAmpere=(double)batteryFr.get.current/10;
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
                        message.cellDJI[i]= (float)batteryFr.get.cell[i]/100;
                        message.seri[i]= batteryFr.get.seriNumber[i];
                    }
                }
                for(uint8_t i=0;i<4;i++)
                {
                    message.version[i] = batteryFr.get.version[i];
                }
            }
            else if(additionFr.set==3)
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
                    message.cellBMS1[i] = bmsFr.get[0].cellVmV[i];
                    //message.cellBMS2[i] = bmsFr.get[1].cellVmV[i];
                }
            }
            else if(additionFr.set==4)
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
                    message.cellBMS2[i] = bmsFr.get[1].cellVmV[i];
                }
            }
            else
            {
                // todo 
            }
            int ret = xQueueSend(QueueHandle, (void *)&message, 0);

            if (ret == pdTRUE) {
                // nếu nhận được data thì xử lý ở đây 
            } else if (ret == errQUEUE_FULL) {
                // nếu nhận không được data thì sử lý ở đây 
            }  // Queue send check
            xSemaphoreGiveRecursive(gui_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        //vTaskDelete(NULL);
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
void pindji(BATTERY_TYPE _battery,lv_obj_t **labelCurrent,lv_obj_t **labelTension,
lv_obj_t **labelWaiting, lv_obj_t **labelTemperature, lv_obj_t **labelChargingTime)
{
    switch(_battery)
    {
        case BATTERY_UNKNOWN:
        {
            DataRandom = 0;
            lv_label_set_text(*labelWaiting, "Đang đợi");
            lv_label_set_text(*labelTension, "0.0");
            lv_label_set_text(*labelCurrent, "0.0");
            lv_label_set_text(*labelTemperature, "0.0");
            lv_label_set_text(*labelChargingTime, "0");
        }break;
        case BATTERY_T30:
        {   
            lv_label_set_text(*labelWaiting, "T30");
        }break;
        case BATTERY_T20P:
        {
            lv_label_set_text(*labelWaiting, "T20P");
        }break;
        case BATTERY_T40:
        {
            lv_label_set_text(*labelWaiting, "T40");
        }break;
        case BATTERY_T25:
        {
            lv_label_set_text(*labelWaiting, "T25");
        }break;
        case BATTERY_T50:
        {
            lv_label_set_text(*labelWaiting, "T50");
        }break;
        default:
            break;
    }
}
void fixLagLCD()
{
    lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);
}

void fastConvertDJI(lv_obj_t *a,char buf[],double data)
{
    sprintf(buf,"%.2f",data);
    lv_label_set_text(a,buf);
    fixLagLCD();
}
void fastConvertForCellDji(lv_obj_t *a, char buf[], float *cellDJI,float *_cell)
{
    sprintf(buf,"%.2f",*cellDJI);
    *_cell = *cellDJI;
    lv_label_set_text(a,buf);
    fixLagLCD();
}

static void Task_Screen_Update(void *pvParameters)
{
    message_t message;
    while(1)
    {
        // Serial.printf("\n[Task_Screen_Update] running on core: %d, Free stack space: %d\n", xPortGetCoreID(), uxTaskGetStackHighWaterMark(NULL));
        if (QueueHandle != NULL) 
        {
            if (xQueueReceive(QueueHandle, &message, portMAX_DELAY))
            {
                //Serial.println("[Task_Screen_Update] The message was successfully received.");
                if (xSemaphoreTakeRecursive(gui_mutex, portMAX_DELAY) == pdTRUE)
                {
                    if (additionFr.set==2)
                    {
                        //lv_obj_clean(labelVonBMS1);
                        if (labelTension != NULL)
                        {
                            fastConvertDJI(labelTension,message.buf,message.dienAp);
                        }

                        if (labelCurrent != NULL)
                        {
                            fastConvertDJI(labelCurrent,message.buf,message.currentAmpere);
                        }
                        if (labelTemperature != NULL)
                        {
                            fastConvertDJI(labelTemperature,message.buf,message.Temperature);
                        }
                        if (labelChargingTime != NULL)
                        {
                            sprintf(message.buf, "%d", message.solanSac);
                            lv_label_set_text(labelChargingTime, message.buf);
                            fixLagLCD();
                        }
                        if (labelWaiting != NULL)
                        {
                            choosePinDji(message.count);
                            pindji(battery,&labelCurrent,&labelTension,&labelWaiting,
                            &labelTemperature,&labelChargingTime);
                            fixLagLCD();
                        }
                        if(version_label != NULL)
                        {
                            char buf[20];
                            sprintf(buf,"vr: %d.%d.%d.%d",message.version[0],message.version[1],message.version[2],message.version[3]);
                            lv_label_set_text(version_label,buf);
                            fixLagLCD();

                        }
                        if(seri_label != NULL)
                        {
                            char buf[19];
                            sprintf(buf,"sn: %c%c%c%c%c%c%c%c%c%c%c%c%c%c",message.seri[0],
                            message.seri[1],message.seri[2],message.seri[3],message.seri[4],message.seri[5],
                            message.seri[6],message.seri[7],message.seri[8],message.seri[9],message.seri[10],
                            message.seri[11],message.seri[12],message.seri[13]);
                            lv_label_set_text(seri_label,buf);
                            fixLagLCD();
                        }
                        if(label_cellone != NULL)
                        {
                            fastConvertForCellDji(label_cellone,message.buf,&message.cellDJI[0],&cellDji[0]);
                        }
                        if(label_celltwo != NULL)
                        {
                            fastConvertForCellDji(label_celltwo,message.buf,&message.cellDJI[1],&cellDji[1]);
                        }
                        if(label_cellthree != NULL)
                        {
                            fastConvertForCellDji(label_cellthree,message.buf,&message.cellDJI[2],&cellDji[2]);
                        }
                        if(label_cellfour != NULL)
                        {
                            fastConvertForCellDji(label_cellfour,message.buf,&message.cellDJI[3],&cellDji[3]);
                        }
                        if(label_cellfive != NULL)
                        {
                            fastConvertForCellDji(label_cellfive,message.buf,&message.cellDJI[4],&cellDji[4]);
                        }
                        if(label_cellsix != NULL)
                        {
                            fastConvertForCellDji(label_cellsix,message.buf,&message.cellDJI[5],&cellDji[5]);
                        }
                        if(label_cellseven != NULL)
                        {
                            fastConvertForCellDji(label_cellseven,message.buf,&message.cellDJI[6],&cellDji[6]);
                        }
                        if(label_celleight != NULL)
                        {
                            fastConvertForCellDji(label_celleight,message.buf,&message.cellDJI[7],&cellDji[7]);
                        }
                        if(label_cellnine != NULL)
                        {
                            fastConvertForCellDji(label_cellnine,message.buf,&message.cellDJI[8],&cellDji[8]);
                        }
                        if(label_cellten != NULL)
                        {
                            fastConvertForCellDji(label_cellten,message.buf,&message.cellDJI[9],&cellDji[9]);
                        }
                        if(label_celleleven != NULL)
                        {
                            fastConvertForCellDji(label_celleleven,message.buf,&message.cellDJI[10],&cellDji[10]);

                        }
                        if(label_celltwelve != NULL)
                        {
                            fastConvertForCellDji(label_celltwelve,message.buf,&message.cellDJI[11],&cellDji[11]);
                        }
                        if(label_cellthirteen != NULL)
                        {
                            fastConvertForCellDji(label_cellthirteen,message.buf,&message.cellDJI[12],&cellDji[12]);
                        }
                        if(label_cellfourteen != NULL)
                        {
                            fastConvertForCellDji(label_cellfourteen,message.buf,&message.cellDJI[13],&cellDji[13]);
                        }
                    }
                    // for BMS1
                    else if(additionFr.set==3)
                    {

                        if(labelVonBMS1 != NULL)
                        {
                            fastConvertDJI(labelVonBMS1,message.buf,message.dienApBms1);
                        }
                        if(labelAmpereBMS1 != NULL)
                        {
                            fastConvertDJI(labelAmpereBMS1,message.buf,message.currentAmpereBms1);
                        }
                        if(labelTemperatureBMS1 != NULL)
                        {
                            fastConvertDJI(labelTemperatureBMS1,message.buf,message.TemperatureBms1);
                        }
                        if(labelChargingTimeBMS1 != NULL)
                        {
                            sprintf(message.buf, "%d", message.solanSacBms1);
                            lv_label_set_text(labelChargingTimeBMS1, message.buf);
                            fixLagLCD();
                        }
                        if(cellBms1One != NULL)
                        {
                            fastConvertForCellDji(cellBms1One,message.buf,&message.cellBMS1[0],&cellBMS1[0]);
                        }
                        if(cellBms1Two != NULL)
                        {
                            fastConvertForCellDji(cellBms1Two,message.buf,&message.cellBMS1[1],&cellBMS1[1]);
                        }
                        if(cellBms1Three != NULL)
                        {
                            fastConvertForCellDji(cellBms1Three,message.buf,&message.cellBMS1[2],&cellBMS1[2]);
                        }
                        if(cellBms1Four != NULL)
                        {
                            fastConvertForCellDji(cellBms1Four,message.buf,&message.cellBMS1[3],&cellBMS1[3]);
                        }
                        if(cellBms1Five != NULL)
                        {
                            fastConvertForCellDji(cellBms1Five,message.buf,&message.cellBMS1[4],&cellBMS1[4]);
                        }
                        if(cellBms1Six != NULL)
                        {
                            fastConvertForCellDji(cellBms1Six,message.buf,&message.cellBMS1[5],&cellBMS1[5]);
                        }
                        if(cellBms1Seven != NULL)
                        {
                            fastConvertForCellDji(cellBms1Seven,message.buf,&message.cellBMS1[6],&cellBMS1[6]);
                        }
                        if(cellBms1Eight != NULL)
                        {
                            fastConvertForCellDji(cellBms1Eight,message.buf,&message.cellBMS1[7],&cellBMS1[7]);
                        }
                        if(cellBms1Nine != NULL)
                        {
                            fastConvertForCellDji(cellBms1Nine,message.buf,&message.cellBMS1[8],&cellBMS1[8]);
                        }
                        if(cellBms1Ten != NULL)
                        {
                            fastConvertForCellDji(cellBms1Ten,message.buf,&message.cellBMS1[9],&cellBMS1[9]);
                        }
                        if(cellBms1Eleven != NULL)
                        {
                            fastConvertForCellDji(cellBms1Eleven,message.buf,&message.cellBMS1[10],&cellBMS1[10]);
                        }
                        if(cellBms1Twelve != NULL)
                        {
                            fastConvertForCellDji(cellBms1Twelve,message.buf,&message.cellBMS1[11],&cellBMS1[11]);
                        }
                        if(cellBms1Thirteen != NULL)
                        {
                            fastConvertForCellDji(cellBms1Thirteen,message.buf,&message.cellBMS1[12],&cellBMS1[12]);
                        }
                        if(cellBms1Fourteen != NULL)
                        {
                            fastConvertForCellDji(cellBms1Fourteen,message.buf,&message.cellBMS1[13],&cellBMS1[13]);
                        }
                        if(cellBms1Fiveteen != NULL)
                        {
                            fastConvertForCellDji(cellBms1Fiveteen,message.buf,&message.cellBMS1[14],&cellBMS1[14]);
                        }
                        if(cellBms1Sixteen != NULL)
                        {
                            fastConvertForCellDji(cellBms1Sixteen,message.buf,&message.cellBMS1[15],&cellBMS1[15]);
                        }
                    }
                    else if(additionFr.set==4)
                    {
                        if(labelVonBMS2 != NULL)
                        {
                            fastConvertDJI(labelVonBMS2,message.buf,message.dienApBms2);
                        }
                        if(labelAmpereBMS2 != NULL)
                        {
                            fastConvertDJI(labelAmpereBMS2,message.buf,message.currentAmpereBms2);
                        }
                        if(labelTemperatureBMS2 != NULL)
                        {
                            fastConvertDJI(labelTemperatureBMS2,message.buf,message.TemperatureBms2);
                        }
                        if(labelChargingTimeBMS2 != NULL)
                        {
                            sprintf(message.buf, "%d", message.solanSacBms2);
                            lv_label_set_text(labelChargingTimeBMS2, message.buf);
                            fixLagLCD();
                        }
                        if(cellBms2One != NULL)
                        {
                            fastConvertForCellDji(cellBms2One,message.buf,&message.cellBMS2[0],&cellBMS2[0]);
                        }
                        if(cellBms2Two != NULL)
                        {
                            fastConvertForCellDji(cellBms2Two,message.buf,&message.cellBMS2[1],&cellBMS2[1]);
                        }
                        if(cellBms2Three != NULL)
                        {
                            fastConvertForCellDji(cellBms2Three,message.buf,&message.cellBMS2[2],&cellBMS2[2]);
                        }
                        if(cellBms2Four != NULL)
                        {
                            fastConvertForCellDji(cellBms2Four,message.buf,&message.cellBMS2[3],&cellBMS2[3]);
                        }
                        if(cellBms2Five != NULL)
                        {
                            fastConvertForCellDji(cellBms2Five,message.buf,&message.cellBMS2[4],&cellBMS2[4]);
                        }
                        if(cellBms2Six != NULL)
                        {
                            fastConvertForCellDji(cellBms1Six,message.buf,&message.cellBMS2[5],&cellBMS2[5]);
                        }
                        if(cellBms2Seven != NULL)
                        {
                            fastConvertForCellDji(cellBms1Seven,message.buf,&message.cellBMS2[6],&cellBMS2[6]);
                        }
                        if(cellBms2Eight != NULL)
                        {
                            fastConvertForCellDji(cellBms2Eight,message.buf,&message.cellBMS2[7],&cellBMS2[7]);
                        }
                        
                        if(cellBms2Nine != NULL)
                        {
                            fastConvertForCellDji(cellBms2Nine,message.buf,&message.cellBMS2[8],&cellBMS2[8]);
                        }
                        if(cellBms2Ten != NULL)
                        {
                            fastConvertForCellDji(cellBms2Ten,message.buf,&message.cellBMS2[9],&cellBMS2[9]);
                        }
                        if(cellBms2Eleven != NULL)
                        {
                            fastConvertForCellDji(cellBms2Eleven,message.buf,&message.cellBMS2[10],&cellBMS2[10]);
                        }
                        if(cellBms2Twelve != NULL)
                        {
                            fastConvertForCellDji(cellBms2Twelve,message.buf,&message.cellBMS2[11],&cellBMS2[11]);
                        }
                        if(cellBms2Thirteen != NULL)
                        {
                            fastConvertForCellDji(cellBms2Thirteen,message.buf,&message.cellBMS2[12],&cellBMS2[12]);
                        }
                        if(cellBms2Fourteen != NULL)
                        {
                            fastConvertForCellDji(cellBms2Fourteen,message.buf,&message.cellBMS2[13],&cellBMS2[13]);
                        }
                        if(cellBms2Fiveteen != NULL)
                        {
                            fastConvertForCellDji(cellBms2Fiveteen,message.buf,&message.cellBMS2[14],&cellBMS2[14]);
                        }
                        if(cellBms2Sixteen != NULL)
                        {
                            fastConvertForCellDji(cellBms2Sixteen,message.buf,&message.cellBMS2[15],&cellBMS2[15]);
                        }
                    }
                    else
                    {
                        //todo
                    }
                    // giải phóng hàng đợi
                    xSemaphoreGiveRecursive(gui_mutex);
                }
            }
            else 
            {
                //Serial.println("[Task_Screen_Update] It was unable to receive data from the Queue.");
            }
        } 
        //vTaskDelete(NULL);
    }
}
/*các biến dùng để chuyển màn với fix lỗi chống đơ dữ liệu hiển thị*/
static bool page2=true;
static bool page3=true;
static bool page4=true;
static bool page5=true;


static void updateLCDScreen(void *pvParameters)
{
    while(1)
    {
        //Serial.printf("\n[updateLCDScreen] running on core: %d, Free stack space: %d\n", xPortGetCoreID(), uxTaskGetStackHighWaterMark(NULL));

        if (xSemaphoreTakeRecursive(gui_mutex, portMAX_DELAY) == pdTRUE)
        {
            if(additionFr.set==2)
            {
                if(page2)
                {
                    _father.stateMachine(additionFr.set);
                    page2=false;
                }
                page4=true;
            }
            else if(additionFr.set==3)
            {
                if(page3)
                {
                    _father.stateMachine(additionFr.set);
                    page3=false;
                }
            }
            else if(additionFr.set==4)
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
            vTaskResume(TaskDataDevice_Handler);
            vTaskResume(TaskMain_Handler);
            vTaskResume(TaskDisplay_Handler);
            vTaskResume(TaskLVGLMAIN_Handler);
            vTaskResume(TaskGetData_Handler);
            vTaskResume(TaskUpdataScreen_Handler);
            xSemaphoreGiveRecursive(gui_mutex);
        }
        
        //Serial.printf("additionFr.set:= %d\n",additionFr.set);
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


float mapFloat(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
    // Tính toán tỷ lệ ánh xạ
    float fromRange = fromHigh - fromLow;
    float toRange = toHigh - toLow;
    float scale = toRange / fromRange;
    
    // Ánh xạ giá trị từ phạm vi đầu vào sang phạm vi đầu ra
    return toLow + ((value - fromLow) * scale);
}


void fourteenCellDJi(float cellDJI,int *data)
{
    // 0->40 
    *data = (int)mapFloat(cellDJI,0,4.25,0,40);
}


/*for bms2 */
void dataCellBMS2one(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[0],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellBMS2two(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[1],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellBMS2three(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[2],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellBMS2four(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[3],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

void dataCellBMS2five(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[4],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellBMS2six(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[5],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellBMS2seven(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[6],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellBMS2eight(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[7],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

void dataCellBMS2nine(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[8],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellBMS2ten(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[9],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellBMS2eleven(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[10],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellBMS2twelve(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[11],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

void dataCellBMS2thirteen(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[12],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellBMS2fourteen(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[13],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellBMS2fifteen(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[14],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellBMS2sixteen(void *bar,int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS2[15],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}

fptr task1DataCellBms2[]={
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
    dataCellBMS2sixteen,
};
/*for bms1 */

void dataCellBMS1one(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[0],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
void dataCellBMS1two(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[1],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
void dataCellBMS1three(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[2],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
void dataCellBMS1four(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[3],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
void dataCellBMS1five(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[4],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
void dataCellBMS1six(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[5],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
void dataCellBMS1seven(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[6],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
void dataCellBMS1eight(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[7],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
void dataCellBMS1nine(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[8],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
void dataCellBMS1ten(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[9],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
void dataCellBMS1eleven(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[10],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
void dataCellBMS1twelve(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[11],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
void dataCellBMS1thirteen(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[12],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
void dataCellBMS1fourteen(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[13],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
void dataCellBMS1fifteen(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[14],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
void dataCellBMS1sixteen(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellBMS1[15],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}

fptr task2DataCellBms1[]={
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
    dataCellBMS1sixteen,
};

void dataCellOne(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[0],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
void dataCellTwo(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[1],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellThree(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[2],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellFour(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[3],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellFive(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[4],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellSix(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[5],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellSeven(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[6],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellEight(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[7],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellnine(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[8],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellTen(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[9],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellEleven(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[10],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellTwelve(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[11],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);

}
void dataCellThirteen(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[12],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}
void dataCellFourteen(void *bar, int32_t temp)
{
    int data;
    fourteenCellDJi(cellDji[13],&data);
    lv_bar_set_value((lv_obj_t*)bar,data,LV_ANIM_ON);
}

fptr task3DataCellDJI[]={
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

void barSmallAll(PageType_t type,lv_obj_t *parent,lv_obj_t *barCell, lv_anim_t *hoatAnh,int32_t label_x,int32_t label_y,uint8_t numberCell)
{
    lv_obj_t ** smallAllCell1;
    lv_obj_t ** smallAllCell2;
    lv_obj_t ** smallAllCell3;
    lv_obj_t ** smallAllCell4;
    lv_obj_t ** smallAllCell5;
    lv_obj_t ** smallAllCell6;
    lv_obj_t ** smallAllCell7;
    lv_obj_t ** smallAllCell8;
    lv_obj_t ** smallAllCell9;
    lv_obj_t ** smallAllCell10;
    lv_obj_t ** smallAllCell11;
    lv_obj_t ** smallAllCell12;
    lv_obj_t ** smallAllCell13;
    lv_obj_t ** smallAllCell14;
    lv_obj_t ** smallAllCell15;
    lv_obj_t ** smallAllCell16;
    if(type == DJI)
    { 
        smallAllCell1 = &label_cellone;
        smallAllCell2 = &label_celltwo;
        smallAllCell3 = &label_cellthree;
        smallAllCell4 = &label_cellfour;
        smallAllCell5 = &label_cellfive;
        smallAllCell6 = &label_cellsix;
        smallAllCell7 = &label_cellseven;
        smallAllCell8 = &label_celleight;
        smallAllCell9 = &label_cellnine;
        smallAllCell10 = &label_cellten;
        smallAllCell11 = &label_celleleven;
        smallAllCell12 = &label_celltwelve;
        smallAllCell13 = &label_cellthirteen;
        smallAllCell14 = &label_cellfourteen;
    }
    else if(type==BMS1)
    {
        smallAllCell1 = &cellBms1One;
        smallAllCell2 = &cellBms1Two;
        smallAllCell3 = &cellBms1Three;
        smallAllCell4 = &cellBms1Four;
        smallAllCell5 = &cellBms1Five;
        smallAllCell6 = &cellBms1Six;
        smallAllCell7 = &cellBms1Seven;
        smallAllCell8 = &cellBms1Eight;
        smallAllCell9 = &cellBms1Nine;
        smallAllCell10 = &cellBms1Ten;
        smallAllCell11 = &cellBms1Eleven;
        smallAllCell12 = &cellBms1Twelve;
        smallAllCell13 = &cellBms1Thirteen;
        smallAllCell14 = &cellBms1Fourteen;
        smallAllCell15 = &cellBms1Fiveteen;
        smallAllCell16 = &cellBms1Sixteen;
    }
    else if(type == BMS2)
    {
        smallAllCell1 = &cellBms2One;
        smallAllCell2 = &cellBms2Two;
        smallAllCell3 = &cellBms2Three;
        smallAllCell4 = &cellBms2Four;
        smallAllCell5 = &cellBms2Five;
        smallAllCell6 = &cellBms2Six;
        smallAllCell7 = &cellBms2Seven;
        smallAllCell8 = &cellBms2Eight;
        smallAllCell9 = &cellBms2Nine;
        smallAllCell10 = &cellBms2Ten;
        smallAllCell11 = &cellBms2Eleven;
        smallAllCell12 = &cellBms2Twelve;
        smallAllCell13 = &cellBms2Thirteen;
        smallAllCell14 = &cellBms2Fourteen;
        smallAllCell15 = &cellBms2Fiveteen;
        smallAllCell16 = &cellBms2Sixteen;
    }
    if(numberCell==1)
    {
        *smallAllCell1 = lv_label_create(parent);
        lv_label_set_text(*smallAllCell1,"4.25");
        lv_obj_align_to(*smallAllCell1,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
        lv_obj_set_style_text_color(*smallAllCell1,lv_color_hex(0x000000),LV_PART_MAIN);
        lv_obj_set_style_text_font(*smallAllCell1,&lv_font_montserrat_18,LV_PART_MAIN);
        #if 1
        if(type==DJI)
            lv_anim_set_exec_cb(hoatAnh,task3DataCellDJI[numberCell-1]);
        else if(type==BMS1)
            lv_anim_set_exec_cb(hoatAnh,task2DataCellBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,task1DataCellBms2[numberCell-1]);
        #endif
    }
    if(numberCell==2)
    {
        *smallAllCell2 = lv_label_create(parent);
        lv_label_set_text(*smallAllCell2,"4.25");
        lv_obj_align_to(*smallAllCell2,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
        lv_obj_set_style_text_color(*smallAllCell2,lv_color_hex(0x000000),LV_PART_MAIN);
        lv_obj_set_style_text_font(*smallAllCell2,&lv_font_montserrat_18,LV_PART_MAIN);
        #if 1
        //lv_anim_init(hoatAnh);
        if(type==DJI)
            lv_anim_set_exec_cb(hoatAnh,task3DataCellDJI[numberCell-1]);
        else if(type==BMS1)
            lv_anim_set_exec_cb(hoatAnh,task2DataCellBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,task1DataCellBms2[numberCell-1]);
        #endif
    }
    if(numberCell==3)
    {
        *smallAllCell3 = lv_label_create(parent);
        lv_label_set_text(*smallAllCell3,"4.25");
        lv_obj_align_to(*smallAllCell3,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
        lv_obj_set_style_text_color(*smallAllCell3,lv_color_hex(0x000000),LV_PART_MAIN);
        lv_obj_set_style_text_font(*smallAllCell3,&lv_font_montserrat_18,LV_PART_MAIN);
        #if 1
        //lv_anim_init(hoatAnh);
        if(type==DJI)
            lv_anim_set_exec_cb(hoatAnh,task3DataCellDJI[numberCell-1]);
        else if(type==BMS1)
            lv_anim_set_exec_cb(hoatAnh,task2DataCellBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,task1DataCellBms2[numberCell-1]);
        #endif
    }
    if(numberCell==4)
    {
        *smallAllCell4 = lv_label_create(parent);
        lv_label_set_text(*smallAllCell4,"4.25");
        lv_obj_align_to(*smallAllCell4,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
        lv_obj_set_style_text_color(*smallAllCell4,lv_color_hex(0x000000),LV_PART_MAIN);
        lv_obj_set_style_text_font(*smallAllCell4,&lv_font_montserrat_18,LV_PART_MAIN);
        #if 1
        //lv_anim_init(hoatAnh);
        if(type==DJI)
            lv_anim_set_exec_cb(hoatAnh,task3DataCellDJI[numberCell-1]);
        else if(type==BMS1)
            lv_anim_set_exec_cb(hoatAnh,task2DataCellBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,task1DataCellBms2[numberCell-1]);
        #endif
    }
    if(numberCell==5)
    {
        *smallAllCell5 = lv_label_create(parent);
        lv_label_set_text(*smallAllCell5,"4.25");
        lv_obj_align_to(*smallAllCell5,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
        lv_obj_set_style_text_color(*smallAllCell5,lv_color_hex(0x000000),LV_PART_MAIN);
        lv_obj_set_style_text_font(*smallAllCell5,&lv_font_montserrat_18,LV_PART_MAIN);
        #if 1
        //lv_anim_init(hoatAnh);
        if(type==DJI)
            lv_anim_set_exec_cb(hoatAnh,task3DataCellDJI[numberCell-1]);
        else if(type==BMS1)
            lv_anim_set_exec_cb(hoatAnh,task2DataCellBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,task1DataCellBms2[numberCell-1]);
        #endif
    }
    if(numberCell==6)
    {
        *smallAllCell6 = lv_label_create(parent);
        lv_label_set_text(*smallAllCell6,"4.25");
        lv_obj_align_to(*smallAllCell6,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
        lv_obj_set_style_text_color(*smallAllCell6,lv_color_hex(0x000000),LV_PART_MAIN);
        lv_obj_set_style_text_font(*smallAllCell6,&lv_font_montserrat_18,LV_PART_MAIN);
        #if 1
        //lv_anim_init(hoatAnh);
        if(type==DJI)
            lv_anim_set_exec_cb(hoatAnh,task3DataCellDJI[numberCell-1]);
        else if(type==BMS1)
            lv_anim_set_exec_cb(hoatAnh,task2DataCellBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,task1DataCellBms2[numberCell-1]);
        #endif
    }
    if(numberCell==7)
    {
        *smallAllCell7 = lv_label_create(parent);
        lv_label_set_text(*smallAllCell7,"4.25");
        lv_obj_align_to(*smallAllCell7,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
        lv_obj_set_style_text_color(*smallAllCell7,lv_color_hex(0x000000),LV_PART_MAIN);
        lv_obj_set_style_text_font(*smallAllCell7,&lv_font_montserrat_18,LV_PART_MAIN);
        #if 1
        //lv_anim_init(hoatAnh);
        if(type==DJI)
            lv_anim_set_exec_cb(hoatAnh,task3DataCellDJI[numberCell-1]);
        else if(type==BMS1)
            lv_anim_set_exec_cb(hoatAnh,task2DataCellBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,task1DataCellBms2[numberCell-1]);
        #endif
    }
    if(numberCell==8)
    {
        *smallAllCell8 = lv_label_create(parent);
        lv_label_set_text(*smallAllCell8,"4.25");
        lv_obj_align_to(*smallAllCell8,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
        lv_obj_set_style_text_color(*smallAllCell8,lv_color_hex(0x000000),LV_PART_MAIN);
        lv_obj_set_style_text_font(*smallAllCell8,&lv_font_montserrat_18,LV_PART_MAIN);
        #if 1
        //lv_anim_init(hoatAnh);
        if(type==DJI)
            lv_anim_set_exec_cb(hoatAnh,task3DataCellDJI[numberCell-1]);
        else if(type==BMS1)
            lv_anim_set_exec_cb(hoatAnh,task2DataCellBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,task1DataCellBms2[numberCell-1]);
        #endif
    }
    if(numberCell==9)
    {
        *smallAllCell9 = lv_label_create(parent);
        lv_label_set_text(*smallAllCell9,"4.25");
        lv_obj_align_to(*smallAllCell9,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
        lv_obj_set_style_text_color(*smallAllCell9,lv_color_hex(0x000000),LV_PART_MAIN);
        lv_obj_set_style_text_font(*smallAllCell9,&lv_font_montserrat_18,LV_PART_MAIN);
        #if 1
        //lv_anim_init(hoatAnh);
        if(type==DJI)
            lv_anim_set_exec_cb(hoatAnh,task3DataCellDJI[numberCell-1]);
        else if(type==BMS1)
            lv_anim_set_exec_cb(hoatAnh,task2DataCellBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,task1DataCellBms2[numberCell-1]);
        #endif
    }
    if(numberCell==10)
    {
        *smallAllCell10 = lv_label_create(parent);
        lv_label_set_text(*smallAllCell10,"4.25");
        lv_obj_align_to(*smallAllCell10,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
        lv_obj_set_style_text_color(*smallAllCell10,lv_color_hex(0x000000),LV_PART_MAIN);
        lv_obj_set_style_text_font(*smallAllCell10,&lv_font_montserrat_18,LV_PART_MAIN);
        #if 1
        //lv_anim_init(hoatAnh);
        if(type==DJI)
            lv_anim_set_exec_cb(hoatAnh,task3DataCellDJI[numberCell-1]);
        else if(type==BMS1)
            lv_anim_set_exec_cb(hoatAnh,task2DataCellBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,task1DataCellBms2[numberCell-1]);
        #endif
    }
    if(numberCell==11)
    {
        *smallAllCell11 = lv_label_create(parent);
        lv_label_set_text(*smallAllCell11,"4.25");
        lv_obj_align_to(*smallAllCell11,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
        lv_obj_set_style_text_color(*smallAllCell11,lv_color_hex(0x000000),LV_PART_MAIN);
        lv_obj_set_style_text_font(*smallAllCell11,&lv_font_montserrat_18,LV_PART_MAIN);
        #if 1
        //lv_anim_init(hoatAnh);
        if(type==DJI)
            lv_anim_set_exec_cb(hoatAnh,task3DataCellDJI[numberCell-1]);
        else if(type==BMS1)
            lv_anim_set_exec_cb(hoatAnh,task2DataCellBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,task1DataCellBms2[numberCell-1]);
        #endif
    }
    if(numberCell==12)
    {
        *smallAllCell12 = lv_label_create(parent);
        lv_label_set_text(*smallAllCell12,"4.25");
        lv_obj_align_to(*smallAllCell12,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
        lv_obj_set_style_text_color(*smallAllCell12,lv_color_hex(0x000000),LV_PART_MAIN);
        lv_obj_set_style_text_font(*smallAllCell12,&lv_font_montserrat_18,LV_PART_MAIN);
        #if 1
        //lv_anim_init(hoatAnh);
        if(type==DJI)
            lv_anim_set_exec_cb(hoatAnh,task3DataCellDJI[numberCell-1]);
        else if(type==BMS1)
            lv_anim_set_exec_cb(hoatAnh,task2DataCellBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,task1DataCellBms2[numberCell-1]);
        #endif
    }
    if(numberCell==13)
    {
        *smallAllCell13 = lv_label_create(parent);
        lv_label_set_text(*smallAllCell13,"4.25");
        lv_obj_align_to(*smallAllCell13,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
        lv_obj_set_style_text_color(*smallAllCell13,lv_color_hex(0x000000),LV_PART_MAIN);
        lv_obj_set_style_text_font(*smallAllCell13,&lv_font_montserrat_18,LV_PART_MAIN);
        #if 1
        //lv_anim_init(hoatAnh);
        if(type==DJI)
            lv_anim_set_exec_cb(hoatAnh,task3DataCellDJI[numberCell-1]);
        else if(type==BMS1)
            lv_anim_set_exec_cb(hoatAnh,task2DataCellBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,task1DataCellBms2[numberCell-1]);
        #endif
    }
    if(numberCell==14)
    {
        *smallAllCell14 = lv_label_create(parent);
        lv_label_set_text(*smallAllCell14,"4.25");
        lv_obj_align_to(*smallAllCell14,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
        lv_obj_set_style_text_color(*smallAllCell14,lv_color_hex(0x000000),LV_PART_MAIN);
        lv_obj_set_style_text_font(*smallAllCell14,&lv_font_montserrat_18,LV_PART_MAIN);
        #if 1
        //lv_anim_init(hoatAnh);
        if(type==DJI)
            lv_anim_set_exec_cb(hoatAnh,task3DataCellDJI[numberCell-1]);
        else if(type==BMS1)
            lv_anim_set_exec_cb(hoatAnh,task2DataCellBms1[numberCell-1]);
        else 
            lv_anim_set_exec_cb(hoatAnh,task1DataCellBms2[numberCell-1]);
        #endif
    }
    if(type == BMS1 || type == BMS2)
    {
        if(numberCell==15)
        {
            *smallAllCell15 = lv_label_create(parent);
            lv_label_set_text(*smallAllCell15,"4.25");
            lv_obj_align_to(*smallAllCell15,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
            lv_obj_set_style_text_color(*smallAllCell15,lv_color_hex(0x000000),LV_PART_MAIN);
            lv_obj_set_style_text_font(*smallAllCell15,&lv_font_montserrat_18,LV_PART_MAIN);
            #if 1
            //lv_anim_init(hoatAnh);
            if(type==DJI)
                lv_anim_set_exec_cb(hoatAnh,task3DataCellDJI[numberCell-1]);
            else if(type==BMS1)
                lv_anim_set_exec_cb(hoatAnh,task2DataCellBms1[numberCell-1]);
            else 
                lv_anim_set_exec_cb(hoatAnh,task1DataCellBms2[numberCell-1]);
            #endif
        }
        if(numberCell==16)
        {
            *smallAllCell16 = lv_label_create(parent);
            lv_label_set_text(*smallAllCell16,"4.25");
            lv_obj_align_to(*smallAllCell16,barCell,LV_ALIGN_OUT_BOTTOM_MID,label_x,label_y);
            lv_obj_set_style_text_color(*smallAllCell16,lv_color_hex(0x000000),LV_PART_MAIN);
            lv_obj_set_style_text_font(*smallAllCell16,&lv_font_montserrat_18,LV_PART_MAIN);
            #if 1
            //lv_anim_init(hoatAnh);
            if(type==DJI)
                lv_anim_set_exec_cb(hoatAnh,task3DataCellDJI[numberCell-1]);
            else if(type==BMS1)
                lv_anim_set_exec_cb(hoatAnh,task2DataCellBms1[numberCell-1]);
            else 
                lv_anim_set_exec_cb(hoatAnh,task1DataCellBms2[numberCell-1]);
            #endif
        }
    }
    
}

// Hàm tiện ích để tạo một thanh bar và nhãn
void create_bar_with_label(lv_obj_t *parent, lv_style_t *style_bg, lv_style_t *style_indic, int32_t x, int32_t y, int32_t min, 
    int32_t max,int32_t label_x,int32_t label_y, int32_t initial_value, int32_t anim_duration,uint8_t numberCell,PageType_t type)
{
    lv_obj_t **barCell;

    if(type == DJI) barCell = &barSmallDji;
    else if(type == BMS1) barCell = &barSmallBms1;
    else barCell = &barSmallBms2;
    // Tạo thanh bar
    *barCell = lv_bar_create(parent);
    lv_obj_add_style(*barCell, style_bg, 0);
    lv_obj_add_style(*barCell, style_indic, LV_PART_INDICATOR);
    lv_obj_set_size(*barCell, 20, 50);
    lv_obj_align(*barCell, LV_ALIGN_BOTTOM_LEFT, x, y);
    lv_bar_set_range(*barCell, min, max);
    lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
    lv_anim_t b;
    lv_anim_init(&b);
    if(type==DJI)
    {
        barSmallAll(DJI,parent,*barCell,&b,label_x,label_y,numberCell);
    }
    else if(type == BMS1)
    {
        barSmallAll(BMS1,parent,*barCell,&b,label_x,label_y,numberCell);
    }
    else if(type == BMS2)
    {
        barSmallAll(BMS2,parent,*barCell,&b,label_x,label_y,numberCell);
    }
    
    lv_anim_set_time(&b, anim_duration);
    lv_anim_set_playback_time(&b, anim_duration);
    lv_anim_set_var(&b, *barCell);
    lv_anim_set_values(&b, min, max);
    lv_anim_set_repeat_count(&b, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&b);

    // Đặt giá trị ban đầu cho thanh bar
    lv_bar_set_value(*barCell, initial_value, LV_ANIM_OFF);

    // lv_anim_del(&b,task3DataCellDJI[0]);
    // lv_anim_del(&b,task3DataCellDJI[1]);
    for(uint8_t i=0;i<FOURTEEN_CELL;i++)
    {
        lv_anim_del(&b,task3DataCellDJI[i]);
    }
    for(uint8_t i=0;i<SIXTEEN_CELL;i++)
    {
        lv_anim_del(&b,task2DataCellBms1[i]);
    }
    for(uint8_t i=0;i<SIXTEEN_CELL;i++)
    {
        lv_anim_del(&b,task1DataCellBms2[i]);
    }
    if(type==DJI)
    {
        
    }
    else if(type == BMS1)
    {
        //lv_obj_clean(barSmallDji);
        // lv_obj_del(barSmallDji);
    }
    else
    {
        //lv_obj_clean(barSmallBms1);
        // lv_obj_del(barSmallBms1);
    }
    //lv_obj_clean(barCell);
    //lv_obj_clean(parent);
    
}

void showCellpin(int x, const int distance,showCell _cell,PageType_t type)
{   
    for(uint8_t i=0;i<lengCleanLvStyle;i++)
        cleanLvStyle[i];
    lv_style_init(&style_indic);
    lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
    lv_style_set_radius(&style_indic, 2);
    lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_GREEN));
    lv_style_set_bg_grad_color(&style_indic, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_bg_grad_dir(&style_indic, LV_GRAD_DIR_VER);

    
    lv_style_init(&style_bg);
    lv_style_set_border_width(&style_bg, 2);
    lv_style_set_pad_all(&style_bg, 2); /*To make the indicator smaller*/
    lv_style_set_radius(&style_bg, 2);
    lv_style_set_anim_time(&style_bg, 1000);
    lv_style_set_bg_opa(&style_bg, LV_OPA_TRANSP);

    for(uint8_t i=0;i<_cell;i++)
    {
            create_bar_with_label(lv_scr_act(), &style_bg, &style_indic, 
                                                        x + i*distance, toadoY, 0, 40, 
                                                        10,0, 0, 3,i+1,type);
    }
}


void update_image_color(lv_obj_t *img, lv_color_t color, lv_opa_t intensity)
{
    lv_obj_set_style_img_recolor_opa(img, intensity, 0);
    lv_obj_set_style_img_recolor(img, color, 0);
}


void sangMoLogo()
{
     /*KHỞI TẠO STYLE BASE */
    
    lv_style_t style_base;
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
    //lv_mem_free(screen);
    
}   
