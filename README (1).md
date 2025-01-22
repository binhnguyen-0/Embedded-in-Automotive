
# BÀI TẬP LÝ THUYẾT

# BÀI 1: FIRST PROJECT WITH KEILC
## 1, Giới thiệu sơ qua về KeilC
- KeilC là 1 IDE hỗ trợ viết, biên dịch, debug code và upload chương trình vào MCU.
- Có hỗ trợ tính năng mô phỏng giúp chạy thử code mà không cần liên kết với phần cứng.
- Hỗ trợ nhiều dòng vi điều khiển như 8051, ARM, AVR, ...
- Trình biên dịch hỗ trợ chuẩn C/C++.
## 2, Lập trình thanh ghi

Đề bài - Nháy led PC13 - để lập trình nháy led PC13 chúng ta cần phải xác định được thanh ghi nào cấp clock điều khiển cho ngoại vi GPIO, và 1 thanh ghi nữa dùng để cấu hình và điều khiển cho GPIO mà ta muốn (đọc RM của MCU).

### a) Thanh ghi RCC (Reset & Clock CONTROL):

#### Trong thanh ghi RCC này, chúng ta sẽ tìm đến thanh ghi bật clock ngoại vi APB2 - APB2 PERIPHERAL CLOCK ENABLE:
- Địa chỉ offset: 0x18
- Trong thanh ghi có các bit từ 2 -> 8 là các bit bật xung cho các IOP từ A -> G, vậy chúng ta sẽ thực hiện bật các IOP bằng cách set bit tương ứng lên 1.
```
#define RCC_APB2ENR *((unsigned int *)0x40021018)
RCC_APB2ENR |= (1 << 4); // set bit 4 - IOPCEN lên 1
``` 
#### --> Đầu tiên định nghĩa địa chỉ cơ bản của thanh ghi RCC_APB2ENR rồi sau đó có thể truy cập vào thanh ghi đó để set bit 4 - IOPCEN lên 1: bật cổng C.
### b) Thanh ghi GPIO (GPIO Register)

#### Trong thanh ghi này, chúng ta sẽ tìm đến các thanh ghi cấu hình, thanh ghi dữ liệu để sử dụng:
##### Thanh ghi GPIOx_CRH (Port configuration register low):
- Địa chỉ offset: 0x04
- Đây là thanh ghi cấu hình cho các chân 8 -> 15 của 1 cổng GPIO, và ta phải truy cập theo 1 cặp bit để cấu hình MODE và CNF.
##### Thanh ghi GPIOx_ODR (Port output data register):
- Địa chỉ offset: 0x0C
- Đây là thanh ghi dùng để điều khiển mức logic của các chân ở chế độ đầu ra.
```
#define GPIOC_CRH (*(unsigned int *)0x40011004)
#define GPIOC_ODR *((unsigned int *)0x4001100C)
```
#### --> Định nghĩa địa chỉ cơ bản của thanh ghi GPIOC_CRH, GPIOC_ODR rồi sau đó có thể truy cập vào thanh ghi đó.
### c) Source code "Nháy led PC13":
```
#define RCC_APB2ENR *((unsigned int *)0x40021018)
#define GPIOC_CRH *((unsigned int *)0x40011004)
#define GPIOC_ODR *((unsigned int *)0x4001100C)

int main(void)
{
    /* Bật clock điều khiển APB2ENR cho GPIOC */
    RCC_APB2ENR |= (1 << 4);
    /* Cấu hình cho GPIOC */
    GPIOC_CRH &= ~((1 << 22) | (1 << 23)); // Chọn chế độ 00: output push-pull
    GPIOC_CRH |= (1 << 20) | (1 << 21);    // Chọn chế độ 11:  Output mode, 50MHz

    while(1)
    {
        GPIOC_ODR |= (1 << 13);
        for (unsigned int i = 0; i < 1000000; i++);
        GPIOC_ODR &= ~(1 << 13);
        for (unsigned int i = 0; i < 1000000; i++);
    }
    return 0;
}
```
## 3, Dùng API:

Đề bài - "Đọc trạng thái nút nhấn", tương tự như trên chúng ta cũng sẽ bật clock
điều khiển và cấu hình GPIO, nhưng ở bài này ta sẽ sử dụng API để viết code và sử
dụng struct để truy cập các thanh ghi.

#### Đầu tiên, tạo RCC và GPIO struct gồm các thanh ghi thành viên của 2 thanh ghi chính này để có thể truy cập nhanh hơn:
```
/* Tạo RCC struct */
typedef struct
{
    unsigned int CR;
    unsigned int CFGR;
    unsigned int CIR;
    unsigned int APB2RSTR;
    unsigned int APB1RSTR;
    unsigned int AHBENR;
    unsigned int APB2ENR;
    unsigned int APB1ENR;
    unsigned int BDCR;
    unsigned int CSR;
}RCC_TypeDef;

/* Tạo GPIO struct */
typedef struct
{
    unsigned int CRL;
    unsigned int CRH;
    unsigned int IDR;
    unsigned int ODR;
    unsigned int BSRR;
    unsigned int BRR;
    unsigned int LCKR;
}GPIO_TypeDef;
```
#### Tiếp theo định nghĩa RCC, GPIOA, GPIOC là các con trỏ trỏ đến địa chỉ cơ bản:
```
/* Địa chỉ cơ bản của của RCC và GPIO */
#define RCC ((RCC_TypeDef *)0x40021000)
#define GPIOA ((GPIO_TypeDef *)0x40010800)
#define GPIOC ((GPIO_TypeDef *)0x40011000)
```
#### Cách truy cập thanh ghi RCC:
```
/* Cấp clock điều khiển APB2ENR cho GPIOC */
RCC->APB2ENR |= (1 << 4);
```
#### Tiếp theo, ta sẽ sử dụng API để lập trình 1 hàm để set và clear giá trị trên 1 chân:
```
void WritePin (GPIO_TypeDef *GPIO_Port, unsigned char Pin, unsigned char state)
{
if (state == 1)
GPIO_Port->ODR |= (1 << Pin);
else
GPIO_Port->ODR &= ~(1 << Pin);
}
```
#### Cách set bit cho PC13 sử dụng API:
```
/* Ghi chân P13 của cổng C lên 1 */
WritePin(GPIOC,13,1);
```
#### Tiếp đến là cấu hình GPIOA để nhận tín hiệu bên ngoài từ nút nhấn:
```
/* Bật xung điều khiển APB2ENR cho GPIOA */
RCC->APB2ENR |= (1 << 2);
/* Cấu hình GPIOA là chế độ Input pull up/pull down */
GPIOA->CRL &= ~((1 << 0) | (1 << 1) | (1 << 2)); // MODE[1:0] = 00: Input mode
GPIOA->CRL |= (1 << 3); // CNF bit 2 và 3 = 10: Input pullup/pulldown
GPIOA->ODR |= (1 << 0); // Chốt pullup
```
#### --> Khi để chế độ là pull up:
- Khi nhấn nút: PA0 sẽ được clear về 0 (vì nút ấn nối với GND)
- Khi không nhấn nút: PA0 bằng 1 (vì PA0 được kéo lên mức cao)
#### Sau đó truy cập thanh ghi IDR để kiểm tra xem là tín hiệu bên ngoài thay đổi như thế nào:
```
/* Đọc IDR0 khi thay đổi trạng thái của nút nhấn */
if ((GPIOA->IDR & (1 << 0)) == 0) 
{
WritePin(GPIOC,13,0);
}
else
{
WritePin(GPIOC,13,1);
}
```
#### Source code "Đọc trạng thái nút nhấn":
```

/* Tạo RCC struct */
typedef struct
{
    unsigned int CR;
    unsigned int CFGR;
    unsigned int CIR;
    unsigned int APB2RSTR;
    unsigned int APB1RSTR;
    unsigned int AHBENR;
    unsigned int APB2ENR;
    unsigned int APB1ENR;
    unsigned int BDCR;
    unsigned int CSR;
}RCC_TypeDef;

/* Tạo GPIO struct */
typedef struct
{
    unsigned int CRL;
    unsigned int CRH;
    unsigned int IDR;
    unsigned int ODR;
    unsigned int BSRR;
    unsigned int BRR;
    unsigned int LCKR;
}GPIO_TypeDef;

/* Địa chỉ cơ bản của của RCC và GPIO */
#define RCC ((RCC_TypeDef *)0x40021000)
#define GPIOA ((GPIO_TypeDef *)0x40010800)
#define GPIOC ((GPIO_TypeDef *)0x40011000)

/* Hàm ghi giá trị chân cho GPIO dùng API */
void WritePin (GPIO_TypeDef *GPIO_Port, unsigned char Pin, unsigned char state)
{
    if (state == 1)
    GPIO_Port->ODR |= (1 << Pin);
    else
    GPIO_Port->ODR &= ~(1 << Pin);
}

/* Hàm cấu hình GPIOA và GPIOC */
void ConfigureGPIO(void)
{
    // Bật clock điều khiển cho IOPA và IOPC
    RCC->APB2ENR |= (1 << 2) | (1 << 4);
    // Cấu hình PA0 là chế độ Input pull up/pull down
    GPIOA->CRL &= ~((1 << 0) | (1 << 1) | (1 << 2)); // MODE = 00: Input mode
    GPIOA->CRL |= (1 << 3); // CNF bit 2 và 3 = 10: Input pullup/pulldown
    GPIOA->ODR |= (1 << 0); // Chốt pullup
    // Cấu hình PC13 là chế độ Output push-pull
    GPIOC->CRH &= ~((1 << 22) | (1 << 23)); // CNF13 - 00: output push-pull
    GPIOC->CRH |= (1 << 20) | (1 << 21); // MODE13 - 11: ouput mode, 50MHz
}
int main(void)
{
    // Gọi hàm cấu hình
    ConfigureGPIO();

    while(1)
    {
        // Kiểm tra IDR0 là 0 hay 1
        if ((GPIOA->IDR & (1 << 0)) == 0)
        {
            WritePin(GPIOC,13,0);
        }
        else
        {
            WritePin(GPIOC,13,1);
        }
    }
    return 0;
}
```

# BÀI 2: GPIO

## 1, Giới thiệu qua về SPL
- SPL - Standard Peripherals Library là 1 thư viện chuẩn cho các ngoại vi do hãng STMicroelectronics cung cấp hỗ trợ cho việc lập trình vi điều khiển STM32. 
- SPL giúp đơn giản hóa việc lập trình và cấu hình các ngoại vi, cung cấp các hàm API để giúp người dùng dễ dàng truy cập thanh ghi mà không cần phải lập trình trực tiếp cho thanh ghi.
## 2, Giới thiệu về GPIO:
- GPIO là một trong các ngoại vi của vi điều khiển STM32, và trong bài này ta sẽ cấu hình cho ngoại vi GPIO bằng SPL.
- Trong SPL, các thanh ghi chức năng GPIO, các thuộc tính như chế độ, tốc độ hay là các chân của GPIO đều đã được tổ chức thành 1 struct để dễ dàng truy cập (stm32f10x_gpio.h).
### a) Cấu hình cho RCC cấp clock điều khiển cho GPIO:
#### Trong thư viện  stm32f10x_rcc.h cung cấp các Marco để định nghĩa các ngoại vi trong đó có GPIO, và trong thư viện stm32f10x_rcc.c cung cấp các hàm API để dễ dàng cấu hình cho RCC.
- Các hàm cơ bản sử dụng để cấp clock là:
```
RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FuntionalState NewState)
hoặc
RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FuntionalState NewState)
```
#### --> Hai hàm trên sử dụng để cấp clock cho bus APB1 và APB2. Ở tham số đầu là các ngoại vi đã được định nghĩa trước và ta chỉ cần đưa ngoại vi mong muốn vào. Tham số thứ 2 là trạng thái để bật hay tắt cấp clock: ENABLE và DISABLE.
#### Cách cấp clock cho GPIO C:
```
/* Hàm cấu hình RCC */
void RCC_Config(void){
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}
```
### b) Cấu hình cho GPIO:
#### Cũng như RCC, để cấp hình cho GPIO mà sử dụng SPL thì ta truy cập vào 2 thư viện `stm32f10x_gpio.h` và `stm32f10x_gpio.c` để xem các hàm được thiết kế với các mục đích khác nhau.
#### Cách cấu hình GPIO cho chân C:
```
/* Hàm cấu hình cho GPIO */
void GPIO_Config(void){
// Khai báo biến với kiểu dữ liệu GPIO_InitTypeDef để truy cập các member
GPIO_InitTypeDef GPIO_InitStructure;
// Sau đó truy cập các thuộc tính
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// Dùng hàm GPIO_Init để chốt các thuộc tính và chân C
GPIO_Init(GPIOC, &GPIO_InitStructure);
}
```
c) Các hàm API thường sử dụng trong `stm32f10x_gpio.c`:
- `GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)`: đặt 1 hoặc nhiều chân ở mức cao.
- `GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)`: đặt 1 hoặc nhiều chân ở mức thấp.
- `GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal)`: đặt 1 chân  ở mức cao hoặc thấp.
- `GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal)`: ghi giá trị 16 bit cho toàn bộ chân trong 1 GPIOx nào đó.
- `GPIO_ReadInputData(GPIO_TypeDef* GPIOx)`: đọc giá trị của tất cả các chân (16 bit) cho 1 GPIOx (cấu hình Input).
- `GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)`: đọc trạng thái của 1 chân trong GPIOx (cấu hình Input).
- `GPIO_ReadOutputData(GPIO_TypeDef* GPIOx)`: đọc giá trị của tất cả các chân (16 bit) cho 1 GPIOx (cấu hình Output).
- `GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)`: đọc trạng thái của 1 chân trong GPIOx (cấu hình Output).
## 3, Source code:
### a) Nháy LED chân PC13:
```
/* Hàm cấu hình RCC */
void RCC_Config(void)
{
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}
/* Hàm cấu hình GPIO */
void GPIO_Config(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOC, &GPIO_InitStructure);
}
/* Hàm main nháy LED */
int main(void)
{
    RCC_Config();
    GPIO_Config();
    while(1)
    {
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
        for (unsigned int i = 0; i < 1000000; i++);
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
        for (unsigned int i = 0; i < 1000000; i++);
    }
}
```
### b) Nháy đuổi LED:
```
/* Hàm cấu hình RCC */
void RCC_Config(void){
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}
/* Hàm cấu hình GPIO */
void GPIO_Config(void){
GPIO_InitTypeDef GPIO_InitStructure;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOC, &GPIO_InitStructure);
}
/* Hàm đuổi LED */
void chasing_LED(uint8_t chasing_Times)
{
    uint16_t value_LED;
    for (int j = 0; j < chasing_Times; j++)
    {
        // Khởi tạo 1 giá trị 16 bit
        value_LED = 0x0010;
        // Nháy led bắt đầu từ bit số 4 đến bit số 8 (tương đương đèn nháy bắt đầu
từ chân 5 -> 8)
        for (int i = 0; i < 4; i++)
        {
            value_LED <<= 1;
            GPIO_Write(GPIOC, value_LED);
            for (unsigned int i = 0; i < 1000000; i++);
        }
    }
}
/* Hàm main đuổi LED */
int main(void)
{
    RCC_Config();
    GPIO_Config();
    while(1)
    {
        // 4 lần nháy đuổi LED
        chasing_LED(4)
        // Nháy xong 4 lần thì dừng
        break;
    }
}
```
### c) Ví dụ sử dụng nút ấn để bật tắt LED PC13:
```
/* Cấu hình RCC */
void RCC_Config(void
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);
}
/* Cấu hình GPIO */
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // Cấu hình cho GPIOC chân PC13
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    // Cấu hình cho GPIOA chân PA0
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/* Hàm main cho nút nhấn */
int main(void)
{
    RCC_Config();
    GPIO_Config();
    while(1)
    {
        // Khi nhấn nút thì đi vào phần body
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET)
        {
            // Khi nhả nút thì đi đến dòng code tiếp theo
            while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == Bit_RESET);
            // Đảo trạng thái hiện tại của LED PC13
            if (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13) == Bit_SET)
            {
                GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
            } else 
            {
                GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
            }
        }
    }
}
```
# Bài 3: INTERRUPT - TIMER

## 1, Giới thiệu về ngắt
- Ngắt là 1 sự kiện khẩn cấp xảy ra trong hoặc ngoài vi điều khiển, yêu cầu MCU dừng thực hiện chương trình chính lại và chạy đến thực thi sự kiện ngắt trước.
- Khi có một sự kiện ngắt nào đó xảy ra thì chương trình dừng và lưu giá trị mà thanh ghi PC hiện đang chỉ tới vào MSP, và PC sẽ chỉ tới lệnh ngắt ISR để thực thi ngắt đó, -> thực thi xong ngắt ISR thì tiếp theo PC sẽ chỉ tới cái địa chỉ đã được lưu ở trong Stack memory và thực hiện tiếp chương trình chính.
#### --> Ở đây, 
- PC (Program Counter): là 1 thanh ghi chỉ tới địa chỉ của lệnh tiếp theo trong chương trình với mục đích cho CPU biết lệnh tiếp theo mà CPU sẽ thực thi.
- ISR (Interrupt Service Routine): là một hàm sử lý ngắt được vi điều khiển gọi tới khi xảy ra ngắt.
- MSP (Main stack pointer): là thanh ghi lưu trữ dữ liệu tạm thời trong quá trình thực thi chương trình, có biểu hiện như là SP.
2, Giới thiệu về Timer:
- Timer là một ngoại vi có chức năng đếm mỗi chu kỳ xung clock một (đếm tăng lên hoặc giảm xuống).
- Có 7 Timer trong STM32F103C8 gồm: 1 Systic timer, 2 Watchdog timer, và 4 Timer chính: Timer 1, 2, 3, 4.
#### --> Có 3 thứ cần lưu ý:
- Timer clock: TIM1, TIM2, TIM3, TIM4 có clock là 72MHz.
- Prescaler: là bộ chia tần số clock của timer, có giá trị tối đa là unsigned 16 bit - 65535.
- Period: giá trị nạp nào cho timer, tối đa là 65535.
## 3, Cấu hình cho Timer:
#### Ở bài này chúng ta không sử dụng ngắt timer nên ta sẽ để cho bộ đếm đếm tới giá trị max là 65535 (Period). Nghĩa là, mỗi khoảng thời gian sẽ đếm đến 65535 rồi tràn.
```
/* Cấu hình RCC cấp clock cho Timer */
void RCC_Config()
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}
/* Cấu hình cho Timer 2 */
void TIM_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_InitStructure;
    // Cấu hình bộ chia chia cho 1
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    // Cấu hình Timer đếm đến 65535 trong 0,1ms: 
    // 0,0001s = 0,1ms = 1/(72*10^6)*(PSC+1) -> PSC = 7199
    TIM_InitStructure.TIM_Prescaler = 7200 - 1;
    TIM_InitStructure.TIM_Period = 0xFFFF;
    // Cấu hình cho bộ đếm đếm lên
    TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    // Cấu hình cho TIM2
    TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
    // Bật ngoại vi TIM2
    TIM_Cmd(TIM2, ENABLE);
}
/* Hàm delay 1ms */
void delay_ms(uint32_t delay_Time)
{
    // Cài giá trị Timer về 0
    TIM_SetCounter(TIM2,0);
    // Chờ cho đến khi giá trị của bộ đếm lớn hơn delay_Time*10
    while (TIM_Get_Counter(TIM2) < delay_Time * 10);
}
```
# Bài 4: COMMUNICATION PROTOCOLS

## 1, Giới thiệu SPI
### a) Khái niệm
#### SPI là 1 chuẩn giao tiếp nối tiếp, đồng bộ, hoạt động ở chế độ song công (truyền nhận cùng thời điểm) và hoạt động theo dạng Master cấp xung và truyền nhận dữ liệu, Slave nhận xung SCK và truyền nhận dữ liệu, và 1 Master có thể giao tiếp với nhiều Slave.
### b) Đặc điểm
#### Tốc độ truyền cao, tối đa là fPCLK/2 cho cả master và slave.
#### SPI sử dụng 4 đường dây:
- SCK (Serial Clock): Thiết bị Master sẽ tạo xung tín hiệu SCK cung cấp cho Slave.
- MISO (Master In - Slave Out): Tín hiệu gửi từ Slave và nhận bởi Master.
- MOSI (Master Out - Slave In): Tín hiệu gửi từ Master và nhận bởi Slave.
- CS - SS (Chip Select - Slave Select): Master chọn 1 Slave để giao tiếp bằng cách kéo đường CS tương ứng xuống mức Low.
### c) Nguyên lý hoạt động
#### Hoạt động cơ bản của SPI:
![Image](https://github.com/user-attachments/assets/bc35f1bc-8a17-4ec0-8715-251eccf75653)
#### --> Việc truyền dữ liệu trên chân MISO và MOSI diễn ra đồng bộ với SCLK:
- Ở sườn lên 1, bit A0 dịch sang shift register của Slave thông qua MOSI và tương tự bit B0 cũng dịch sang shift register của Master thông qua MISO.
- Những bit còn lại cũng hoạt động tương tự.
#### Slave Select pin - SS:
![Image](https://github.com/user-attachments/assets/b464443f-116f-4769-b04b-c327b1df627b)
- Khi Master muốn giao tiếp với Slave, Master sử dụng 1 chân GPIO để kéo chân SS của Slave xuống mức '0'.
#### -->  Chỉ khi chân SS của Slave được kéo xuống '0' thì các chân MOSI, MISO của Slave mới được kích hoạt, nếu không thì 2 chân này sẽ ở trạng thái trở kháng cao.
#### CPOL & CPHA:
#### Có 4 SPI mode được quyết định bởi 2 tham số CPOL (Clock Polarity) và CPHA (Clock Phase):
- CPOL là tham số quyết định giá trị của SCK khi ở trạng thái rảnh - Idle là Low hay là High.
- CPHA là tham số quyết định xem cạnh thứ nhất hay cạnh thứ 2 của SCK mà Slave sẽ lấy dữ liệu trên đường MOSI và Master lấy dữ liệu trên đườn MISO. 
#### --> 4 chế độ SPI:
![Image](https://github.com/user-attachments/assets/99468976-2378-4e57-8e46-5a9e7eeb010a)
- Mode 0: CPOL = 0 - CPHA = 0: Lấy data ở cạnh lên (cạnh 1) và truyền ở cạnh xuống (cạnh 2).
- Mode 1: CPOL = 0 - CPHA = 1: Lấy data ở cạnh xuống (cạnh 2) và truyền data ở cạnh lên (cạnh 1).
- Mode 2: CPOL = 1 - CPHA = 0: Lấy data ở cạnh xuống (cạnh 1) và truyền data ở cạnh lên (cạnh 2).
- Mode 3: CPOL = 1 - CPHA = 1: Lấy data ở cạnh lên (cạnh 2) và truyền data ở cạnh xuống (cạnh 1).
## 2, Giới thiệu I2C
### a) Khái niệm
#### I2C là chuẩn giao tiếp nối tiếp, đồng bộ, hoạt động ở chế độ bán song công (truyền và nhận không cùng 1 thời điểm), hỗ trợ chế độ đa chủ - nhiều Master giao tiếp với nhiều Slave.
#### Trong I2C, mỗi Slave sẽ có 1 địa chỉ riêng và master sẽ giao tiếp với mỗi slave dựa trên mỗi địa chỉ của slave đó.
### b) Đặc điểm
#### Tốc độ truyền thấp, chậm hơn nhiều so với SPI:
- Standard speed: lên đến 100kHz
- Fast speed: lên đến 400kHz
#### I2C sử dụng 2 đường dây:
- SCL (Serial Clock): Tạo xung clock để đồng bộ việc truyền nhận dữ liệu với Slave.
- SDA (Serial Data): Chân truyền dữ liệu.
#### --> Khi kết nối với bus I2C thì SCL và SDA phải ở chế độ Open-drain và khi ở chế độ idle thì mỗi đường dây cần kết nối 1 điện trở kéo lên nguồn '+'.
### c) Nguyên lý hoạt động
#### Hoạt động cơ bản của I2C:
#### --> Việc truyền dữ liệu diễn ra trên đường SDA:
- Đầu tiên Master tạo ra START condition: bắt đầu quá trình truyền - nhận.
- Sau START condition là ADDRESS phase - 8 bit gốm 7 bit đầu tiên là địa chỉ của Slave và tiếp là 1 bit R/W (R/W = 0: là truyền dữ liệu đến Slave; R/W = 1: là yêu cầu dữ liệu từ Slave).
- Sau 8 bit ADDRESS là 1 bit ACK = 0 - nhận từ Slave: báo hiệu đã nhận được dữ liệu.
- Tiếp theo là 8 bit data: Master truyền hay nhận data tùy thuộc vào bit R/W.
- Sau 8 bit data là 1 bit ACK = 0 - nhận từ Slave nếu R/W = 0 và nhận từ Master nếu R/W = 1.
- Cuối cùng, nếu việc truyền - nhận đã xong thì Master sẽ tạo ra STOP condition: kết thúc truyền - nhận và các Master khác có thể bắt đầu truyền - nhận dữ liệu.
#### Start và Stop condition:
![Image](https://github.com/user-attachments/assets/feac2f38-89e4-49c6-ae0b-914a6d9b27c4)
- Tất cả các giao dịch dữ liệu đều được bắt đầu bằng 1 Start (S) và kết thúc bằng 1 Stop (P).
#### --> Khi SCL ở mức HIGH:
- Sự chuyển đổi logic từ HIGH sang LOW ở đường SDA là Start condition.
- Sự chuyển đổi logic từ LOW sang HIGH ở đường SDA là Stop condition.
#### ACK & NACK (Acknowledgement & Negative Acknowledgement):
- ACK hoặc NACK đứng sau mỗi 1 byte trong I2C để báo hiệu với bên gửi rằng bên nhận đã nhận được dữ liệu.
- ACK = 0: đã nhận được dữ liệu.
- NACK = 1: không nhận được dữ liệu.
## 3, Giới thiệu UART và USART
### a) Khái niệm
#### Nghĩa:
- UART: Universal Asynchronous Receiver/Transmitter - Máy thu/phát không đồng bộ phổ thông.
- USART: Universal Synchronous/Asynchronous Receiver/Transmitter - Máy thu/phát không đồng bộ/đồng bộ phổ thông.
#### --> Hiện tại các vi điều khiển đều có USART thay vì chỉ có UART.
#### UART/USART là 1 giao thức truyền thông phần cứng dùng giao tiếp nối tiếp đồng bộ/không đồng bộ và có thể cấu hình được tốc độ, hoạt động ở chế độ song công (truyền nhận cùng thời điểm).
### b) Đặc điểm
#### Tốc độ truyền là các số chuẩn như 9600, 19200, 38400, 57600, 115200, ... bps. Nếu như ta sử dụng UART thì cả 2 thiết bị nhận và truyền đều phải có cùng baud rate.
#### UART sử dụng 2 đường dây:
- Nhận dữ liệu thông qua chân Rx.
- Truyền dữ liệu đi bằng chân Tx, khi ở trạng thái idle thì Tx được giữ ở mức HIGH.
![Image](https://github.com/user-attachments/assets/9d73e54d-39e8-4b3b-a9f6-859254d2ae63)
### c) Nguyên lý hoạt động
![Image](https://github.com/user-attachments/assets/1104eb04-f9e5-49dd-96e7-3f4705998e29)
#### Hoạt động cơ bản của UART khi nhận và gửi:
- Đầu tiên là bắt đầu bằng một Start bit = 0.
- Tiếp đó là data frame thường chiếm khoảng 5 - 9 bit.
- Theo sau là 1 Parity bit, có thể lựa chọn giữa Parity chẵn hoặc Parity lẻ.
- Cuối cùng, frame kết thúc bằng 1 Stop bit = 1, có thể cấu hình thời lượng cho Stop bit là 1 hay 1,5 hoặc 2 bit.
#### UART Parity bit:
- Thêm bit Parity là 1 phương pháp để phát hiện lỗi khi truyền dữ liệu, có 2 loại là Parity chẵn và Parity lẻ.
#### Even parity:
![Image](https://github.com/user-attachments/assets/3daef399-31f5-438f-8ad5-f98a83c0360d)
- Parity chẵn là tổng số bit '1' trong data frame phải là số CHẴN --> nếu tổng là số LẺ thì bit Parity = 1 hoặc Parity = 0 nếu tổng ban đầu đã là số CHẴN.
#### Odd parity:
![Image](https://github.com/user-attachments/assets/37d31fb6-12a5-47e3-9578-16bd8960c280)
- Parity lẻ là tổng số bit '1' trong data frame phải là số LẺ --> nếu tổng là số CHẴN thì bit Parity = 1 hoặc Parity = 0 nếu tổng ban đầu đã là số LẺ.
