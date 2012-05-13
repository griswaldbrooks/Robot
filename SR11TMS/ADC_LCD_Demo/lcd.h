
void set_ena      (void);
void clr_ena      (void);
void set_rw       (void);
void clr_rw       (void);
void set_rs       (void);
void clr_rs       (void);

int  lcd_status   (void);
int  read_data    (void);
void write_data   (int);
void write_control(int);
void write        (int);
void wait_lcd     (void);
void init_lcd     (void);
void clear_lcd    (void);
void light_on     (void);
void light_off    (void);
void put_cursor   (int x, int y);
void put_char     (int x, int y, int c);
void put_text     (char* t);
void set_io_data  (int d);

void SW_Delay(unsigned long);

//------------------------------------------------------------------------------
// Define CPU Frequency >>here<<
//------------------------------------------------------------------------------
#define CPU_FREQUENCY        600000000

#define DELAY_1US            (unsigned int)(0.000001f * CPU_FREQUENCY / 4)
#define DELAY_10US           (unsigned int)(0.00001f * CPU_FREQUENCY / 4)
#define DELAY_200US          (unsigned int)(0.0003f * CPU_FREQUENCY / 4)
#define DELAY_1MS            (unsigned int)(0.001f * CPU_FREQUENCY / 4)
#define DELAY_10MS           (unsigned int)(0.01f * CPU_FREQUENCY / 4)

#define SQUARE 0xFF
