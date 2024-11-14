Bài 1: First project with KeilC

A. Cấu trúc tổng quan

    - Giới thiệu qua về KeilC
    - Thực hành ví dụ "Nháy led PC13" (lập trình thanh ghi)
    - Thực hành ví dụ "Đọc trạng thái nút nhấn" (dùng API)
    - Kết luận

    1, Giới thiệu sơ qua về KeilC

        - KeilC là 1 IDE hỗ trợ viết, biên dịch, debug code và upload chương trình vào MCU.
        - Có hỗ trợ tính năng mô phỏng giúp chạy thử code mà không cần liên kết với phân
        cứng.
        - Hỗ trợ nhiều dòng vi điều khiển như 8051, ARM, AVR, ...
        - Trình biên dịch hỗ trợ chuẩn C/C++.

    2, Lập trình thanh ghi

        Đề bài - Nháy led PC13 - để lập trình nháy led PC13 chúng ta cần phải xác định
        được thanh ghi nào cấp clock điều khiển cho ngoại vi GPIO, và 1 thanh ghi nữa dùng
        để cấu hình và điều khiển cho GPIO mà ta muốn (đọc RM của MCU).

        a) Thanh ghi RCC (Reset & Clock CONTROL):

            - Trong thanh ghi RCC này, chúng ta sẽ tìm đến thanh ghi bật clock ngoại vi
            APB2 - APB2 PERIPHERAL CLOCK ENABLE:
                +) Địa chỉ offset: 0x18
                +) Trong thanh ghi có các bit từ 2 -> 8 là các bit bật xung cho các IOP từ 
                    A -> G, vậy chúng ta sẽ thực hiện bật các IOP bằng cách set bit tương
                    ứng lên 1
            - Tiếp theo, ta sẽ tiến hành viết source code:
            ```
            #define RCC_APB2ENR *((unsigned int *)0x40021018)
            ``` 
            -> Chúng ta sử dụng chỉ thị define để định nghĩa lại *((unsigned int *)
            0x40021018) thành RCC_APB2ENR để sử dụng thuận tiện hơn khi viết code, lí do
            phải sử dụng:
                +) unsigned int: ta phải chắc chắn rằng giá trị được trỏ tới là kiểu int
                    32 bit không âm, lí do: tránh các vấn đề với số âm, và thanh ghi trên
                    STM32 MCU kích thước 32 bit nên giá trị truy cập là unsigned int với
                    kích thước tương ứng giúp truy cập dữ liệu đúng và đủ.  
                +) Địa chỉ 0x40021018: Ta phải xem bảng "Register boundary addresses"
                    trong mục "Memory map" của RM, để biết được địa chỉ bắt đầu của RCC là
                    0x4002 1000 - 0x4002 13FF, vậy ta sẽ lấy 0x4002 1000 offset với 0x18
                    sẽ được -> 0x4002 1018. 
            - Rồi sau khi đã định nghĩa xong, ta có thể truy cập trực tiếp thanh ghi như
            sau:
            /* Bật clock điều khiển cho GPIOC */
            ```
            RCC_APB2ENR |= (1 << 4); // set bit 4 - IOPCEN lên 1
            ```
        b) Thanh ghi GPIO (GPIO Register)

            - Trong thanh ghi này, chúng ta sẽ tìm đến các thanh ghi cấu hình, thanh ghi
            dữ liệu để sử dụng:
                +) Thanh ghi GPIOx_CRH (Port configuration register low):
                    \ Địa chỉ offset: 0x04
                    \ Đây là thanh ghi cấu hình cho các chân 8 -> 15 của 1 cổng GPIO, và
                        ta phải truy cập thep 1 cặp bit để cấu hình MODE và CNF (chi tiết
                        cách sử dụng ở trong RM).
                +) Thanh ghi GPIOx_ODR (Port output data register):
                    \ Địa chỉ offset: 0x0C
                    \ Đây là thanh ghi dùng để điều khiển mức logic của các chân ở chế độ
                        đầu ra.
            - Tương tự như trên, ta sẽ phải định nghĩa con trỏ để có thể truy cập vào
            thanh ghi:
            ```
            #define GPIOC_CRH (*(unsigned int *)0x40011004)
            #define GPIOC_ODR *((unsigned int *)0x4001100C)
            ```
            -> Định nghĩa xong ta có thể truy cập trực tiếp vào thanh ghi.
        c) Source code "Nháy led PC13":
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
    3, Dùng API:
        Đề bài - "Đọc trạng thái nút nhấn", tương tự như trên chúng ta cũng sẽ bật clock
        điều khiển và cấu hình GPIO, nhưng ở bài này ta sẽ sử dụng API để viết code và sử
        dụng struct để truy cập các thanh ghi.
            - Đầu tiên, ta cũng sẽ phải bật clock điều khiển APB2ENR cho GPIO A và C:
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
            -> Chúng ta tạo ra các struct bao gồm thành viên là tất cả các thanh ghi nằm
            trong RCC và GPIO, đi kèm với typedef để định nghĩa lại tên kiểu dữ liệu
            struct này là RCC_TypeDef và GPIO_TypeDef để có thể truy cập nhanh chóng và
            dễ dàng hơn:
            ```
            /* Địa chỉ cơ bản của của RCC và GPIO */
            #define RCC ((RCC_TypeDef *)0x40021000)
            #define GPIOA ((GPIO_TypeDef *)0x40010800)
            #define GPIOC ((GPIO_TypeDef *)0x40011000)
            ```
            -> Chúng ta định nghĩa 1 identifier RCC là một con trỏ kiểu RCC_TypeDef để trỏ
            đến địa chỉ cơ bản của thanh ghi RCC, các GPIOA và GPIOC cũng như vậy, ta
            có thể truy cập thanh ghi như sau:
            ```
            /* Cấp clock điều khiển APB2ENR cho GPIOC */
            RCC->APB2ENR |= (1 << 4);
            ```
            - Tiếp theo, ta sẽ sử dụng API để lập trình 1 hàm để set và clear giá trị trên
             1 chân:
            ```
            void WritePin (GPIO_TypeDef *GPIO_Port, unsigned char Pin, unsigned char state)
            {
                if (state == 1)
                GPIO_Port->ODR |= (1 << Pin);
                else
                GPIO_Port->ODR &= ~(1 << Pin);
            }
            ```
            -> Ta đã tạo ra 1 hàm ghi giá trị 1 chân giống như trong thư viện API giúp
            code của chúng ta dễ hiểu và dễ sử dụng hơn:
            ```
            /* Ghi chân P13 của cổng C lên 1 */
            WritePin(GPIOC,13,1);
            ```
            - Tiếp đến là cấu hình GPIOA để nhận tín hiệu bên ngoài  từ nút nhấn:
            ```
            /* Bật xung điều khiển APB2ENR cho GPIOA */
            RCC->APB2ENR |= (1 << 2);
            /* Cấu hình GPIOA là chế độ Input pull up/pull down */
            GPIOA->CRL &= ~((1 << 0) | (1 << 1) | (1 << 2)); // MODE[1:0] = 00: Input mode
            GPIOA->CRL |= (1 << 3); // CNF bit 2 và 3 = 10: Input pullup/pulldown
            GPIOA->ODR |= (1 << 0); // Chốt pullup (xem bảng Port bit configuration table)
            ```
            -> Khi để chế độ là pull up:
                +) Khi nhấn nút: PA0 sẽ được clear về 0 (vì nút ấn nối với GND)
                +) Khi không nhấn nút: PA0 bằng 1 (vì PA0 được kéo lên mức cao)
            - Sau đó truy cập thanh ghi IDR để kiểm tra xem là tín hiệu bên ngoài thay đổi
            như thế nào:
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
            - Source code "Đọc trạng thái nút nhấn":
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
    4, Kết luận:
        Trong bài này, ta đã làm quen với KeilC - 1 IDE hỗ trợ viết code, biên dịch, debug
        và upload code cho MCU. Và ta cũng đã thực hành viết code sử dụng trực tiếp các
        thanh ghi trong MCU để có thể hiểu rõ cách hoạt động của chúng, rồi sử dụng API để
        áp dụng vào thực tế khi làm việc để viết code dễ hiểu hơn.








