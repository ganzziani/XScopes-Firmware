;-----------------------------------------------------------------------------;
; Fixed-point FFT routines for megaAVRs                        (C)ChaN, 2005
;-----------------------------------------------------------------------------;
; * This program is opened under license policy of following trems.
;
;  Copyright (C) 2005, ChaN, all right reserved.
;
; * This program is a free software and there is NO WARRANTY.
; * No restriction on use. You can use, modify and redistribute it for
;   personal, non-profit or commercial use UNDER YOUR RESPONSIBILITY.
; * Redistributions of source code must retain the above copyright notice.
;
;-----------------------------------------------------------------------------;
;
; void fft_input (const int16_t *array_src, complex_t *array_bfly);
; void fft_execute (complex_t *array_bfly);
; void fft_output (complex_t *array_bfly, uint16_t *array_dst);
;
;  <array_src>: Wave form to be processed.
;  <array_bfly>: Complex array for butterfly operations.
;  <array_dst>: Spectrum output buffer.
;
; These functions must be called in sequence to do a DFT in FFT algorithm.
; fft_input() fills the complex array with a wave form to prepare butterfly
; operations. A hamming window is applied at the same time.
; fft_execute() executes the butterfly operations.
; fft_output() re-orders the results, converts the complex spectrum into
; scalar spectrum and output it in linear scale.
;
; The number of points FFT_N is defined in "ffft.h" and the value can be
; power of 2 in range of 64 - 1024.
;
;----------------------------------------------------------------------------;
; 16bit fixed-point FFT performance with MegaAVRs
; (Running at 16MHz/internal SRAM)
;
;  Points:   Input, Execute,  Output,    Total:  Throughput
;   64pts:   .17ms,   2.0ms,   1.2ms,    3.4ms:   19.0kpps
;  128pts:   .33ms,   4.6ms,   2.4ms,    7.3ms:   17.5kpps
;  256pts:   .66ms,  10.4ms,   4.9ms,   15.9ms:   16.1kpps
;  512pts:   1.3ms,  23.2ms,   9.7ms,   34.2ms:   14.9kpps
; 1024pts:   2.7ms,  51.7ms,  19.4ms,   73.7ms:   13.9kpps
;----------------------------------------------------------------------------;


.nolist
#define FFFT_ASM
#include "ffft.h"
.list

#if FFT_N == 1024
#define FFT_B 10
#elif FFT_N == 512
#define FFT_B 9
#elif FFT_N == 256
#define FFT_B 8
#elif FFT_N == 128
#define FFT_B 7
#elif FFT_N == 64
#define FFT_B 6
#else
#error Wrong setting of FFT_N.
#endif


;----------------------------------------------------------------------------;
; Constant Tables
tbl_cos_sin:	; Table of {cos(x),sin(x)}, (0 <= x < pi, in FFT_N/2 steps)
    .dc.w    32767, 0, 32757, 804, 32728, 1608, 32678, 2410, 32609, 3212, 32521, 4011, 32412, 4808, 32285, 5602
    .dc.w    32137, 6393, 31971, 7179, 31785, 7962, 31580, 8739, 31356, 9512, 31113, 10278, 30852, 11039, 30571, 11793
    .dc.w    30273, 12539, 29956, 13279, 29621, 14010, 29268, 14732, 28898, 15446, 28510, 16151, 28105, 16846, 27683, 17530
    .dc.w    27245, 18204, 26790, 18868, 26319, 19519, 25832, 20159, 25329, 20787, 24811, 21403, 24279, 22005, 23731, 22594
    .dc.w    23170, 23170, 22594, 23731, 22005, 24279, 21403, 24811, 20787, 25329, 20159, 25832, 19519, 26319, 18868, 26790
    .dc.w    18204, 27245, 17530, 27683, 16846, 28105, 16151, 28510, 15446, 28898, 14732, 29268, 14010, 29621, 13279, 29956
    .dc.w    12539, 30273, 11793, 30571, 11039, 30852, 10278, 31113, 9512, 31356, 8739, 31580, 7962, 31785, 7179, 31971
    .dc.w    6393, 32137, 5602, 32285, 4808, 32412, 4011, 32521, 3212, 32609, 2410, 32678, 1608, 32728, 804, 32757
    .dc.w    0, 32767, -804, 32757, -1608, 32728, -2410, 32678, -3212, 32609, -4011, 32521, -4808, 32412, -5602, 32285
    .dc.w    -6393, 32137, -7179, 31971, -7962, 31785, -8739, 31580, -9512, 31356, -10278, 31113, -11039, 30852, -11793, 30571
    .dc.w    -12539, 30273, -13279, 29956, -14010, 29621, -14732, 29268, -15446, 28898, -16151, 28510, -16846, 28105, -17530, 27683
    .dc.w    -18204, 27245, -18868, 26790, -19519, 26319, -20159, 25832, -20787, 25329, -21403, 24811, -22005, 24279, -22594, 23731
    .dc.w    -23170, 23170, -23731, 22594, -24279, 22005, -24811, 21403, -25329, 20787, -25832, 20159, -26319, 19519, -26790, 18868
    .dc.w    -27245, 18204, -27683, 17530, -28105, 16846, -28510, 16151, -28898, 15446, -29268, 14732, -29621, 14010, -29956, 13279
    .dc.w    -30273, 12539, -30571, 11793, -30852, 11039, -31113, 10278, -31356, 9512, -31580, 8739, -31785, 7962, -31971, 7179
    .dc.w    -32137, 6393, -32285, 5602, -32412, 4808, -32521, 4011, -32609, 3212, -32678, 2410, -32728, 1608, -32757, 804

tbl_bitrev:        ; tbl_bitrev[] = ...
    .dc.b      0, 128,  64, 192,  32, 160,  96, 224,  16, 144,  80, 208,  48, 176, 112, 240
    .dc.b      8, 136,  72, 200,  40, 168, 104, 232,  24, 152,  88, 216,  56, 184, 120, 248
    .dc.b      4, 132,  68, 196,  36, 164, 100, 228,  20, 148,  84, 212,  52, 180, 116, 244
    .dc.b     12, 140,  76, 204,  44, 172, 108, 236,  28, 156,  92, 220,  60, 188, 124, 252
    .dc.b      2, 130,  66, 194,  34, 162,  98, 226,  18, 146,  82, 210,  50, 178, 114, 242
    .dc.b     10, 138,  74, 202,  42, 170, 106, 234,  26, 154,  90, 218,  58, 186, 122, 250
    .dc.b      6, 134,  70, 198,  38, 166, 102, 230,  22, 150,  86, 214,  54, 182, 118, 246
    .dc.b     14, 142,  78, 206,  46, 174, 110, 238,  30, 158,  94, 222,  62, 190, 126, 254
    .dc.b      1, 129,  65, 193,  33, 161,  97, 225,  17, 145,  81, 209,  49, 177, 113, 241
    .dc.b      9, 137,  73, 201,  41, 169, 105, 233,  25, 153,  89, 217,  57, 185, 121, 249
    .dc.b      5, 133,  69, 197,  37, 165, 101, 229,  21, 149,  85, 213,  53, 181, 117, 245
    .dc.b     13, 141,  77, 205,  45, 173, 109, 237,  29, 157,  93, 221,  61, 189, 125, 253
    .dc.b      3, 131,  67, 195,  35, 163,  99, 227,  19, 147,  83, 211,  51, 179, 115, 243
    .dc.b     11, 139,  75, 203,  43, 171, 107, 235,  27, 155,  91, 219,  59, 187, 123, 251
    .dc.b      7, 135,  71, 199,  39, 167, 103, 231,  23, 151,  87, 215,  55, 183, 119, 247
    .dc.b     15, 143,  79, 207,  47, 175, 111, 239,  31, 159,  95, 223,  63, 191, 127, 255

;----------------------------------------------------------------------------;
.global Sin                     ; Return 8bit sine value
.func Sin
Sin:
    clr     r23
    clc
    rol     r24                 ; Calculate correct index
    rol     r23                 ; (The right values are found every 4 bytes in the the tbl_cos_sin table)
    rol     r24
    rol     r23                 ; multiply by 4
    mov     r22,r23
    sbrc    r23, 1              ; angle >=128
    subi    r22, 2              ; substract 128
    ldiw    ZH,ZL, tbl_cos_sin+3
    addw    ZH,ZL, r22,r24
    lpm     r24, z
    sbrc    r23, 1              ; angle >=128
    neg     r24                 ; return -sin(angle-128)
    ret
.endfunc

;----------------------------------------------------------------------------;
.global fft_execute
.func fft_execute
fft_execute:
    pushw   T2H,T2L
    pushw   T4H,T4L
    pushw   T6H,T6L
    pushw   T8H,T8L
    pushw   T10H,T10L
    pushw   T12H,T12L
    pushw   T14H,T14L
    pushw   AH,AL
    pushw   YH,YL

    movw    ZL, EL              ; Z = array_bfly;
    ldiw    EH,EL, 1            ; E = 1;
    ldiw    XH,XL, FFT_N/2      ; X = FFT_N/2;
1:  ldi     AL, 4               ; T12 = E; (angular speed)
    mul     EL, AL              ;
    movw    T12L, T0L           ;
    mul     EH, AL              ;
    add     T12H, T0L           ;/
    movw    T14L, EL            ; T14 = E;
    pushw   EH,EL
    movw    YL, ZL              ; Z = &array_bfly[0];
    mul     XL, AL              ; Y = &array_bfly[X];
    addw    YH,YL, T0H,T0L      ;
    mul     XH, AL              ;
    add     YH, T0L             ;/
    pushw   ZH,ZL
2:  clrw    T10H,T10L           ; T10 = 0 (angle)
    clr     EH                  ; Zero reg.
3:  lddw    AH,AL, Z+0          ; A = *Z - *Y; *Z++ += *Y;
    asrw    AH,AL               ;
    lddw    DH,DL, Y+0          ;
    asrw    DH,DL               ;
    movw    CL, AL              ;
    subw    AH,AL, DH,DL        ;
    addw    CH,CL, DH,DL        ;
    stw     Z+, CH,CL           ; /
    lddw    BH,BL, Z+0          ; B = *Z - *Y; *Z++ += *Y;
    asrw    BH,BL               ;
    lddw    DH,DL, Y+2          ;
    asrw    DH,DL               ;
    movw    CL, BL              ;
    subw    BH,BL, DH,DL        ;
    addw    CH,CL, DH,DL        ;
    stw    Z+, CH,CL            ;/
    movw    T0L, ZL
    ldiw    ZH,ZL, tbl_cos_sin  ; C = cos(T10); D = sin(T10);
    addw    ZH,ZL, T10H,T10L    ;
    lpmw    CH,CL, Z+           ;
    lpmw    DH,DL, Z+           ;/
    movw    ZL, T0L
    FMULS16 T4H,T4L,T2H,T2L, AH,AL, CH,CL    ; *Y++ = A * C + B * D;
    FMULS16 T8H,T8L,T6H,T6L, BH,BL, DH,DL    ;
    addd    T4H,T4L,T2H,T2L, T8H,T8L,T6H,T6L;
    stw     Y+, T4H,T4L            ;/
    FMULS16 T4H,T4L,T2H,T2L, BH,BL, CH,CL     ; *Y++ = B * C - A * D;
    FMULS16 T8H,T8L,T6H,T6L, AH,AL, DH,DL     ;
    subd    T4H,T4L,T2H,T2L, T8H,T8L,T6H,T6L;
    stw     Y+, T4H,T4L         ;/
    addw    T10H,T10L, T12H,T12L        ; T10 += T12; (next angle)
#if FFT_N >= 128
    sbrs    T10H, FFT_B - 7     ; while(T10 < pi)
#else
    sbrs    T10L, FFT_B + 1
#endif
    rjmp    3b                  ;/
    ldi     AL, 4               ; Y += X; Z += X; (skip split segment)
    mul     XL, AL
    addw    YH,YL, T0H,T0L      ;
    addw    ZH,ZL, T0H,T0L      ;
    mul     XH, AL              ;
    add     YH, T0L             ;
    add     ZH, T0L             ;/
    ldi     EL, 1               ; while(--T14)
    subw    T14H,T14L, EH,EL    ;
    rjne    2b                  ;/
    popw    ZH,ZL
    popw    EH,EL
    lslw    EH,EL               ; E *= 2;
    lsrw    XH,XL               ; while(X /= 2)
    adiw    XL, 0               ;
    rjne    1b                  ;/

    popw    YH,YL
    popw    AH,AL
    popw    T14H,T14L
    popw    T12H,T12L
    popw    T10H,T10L
    popw    T8H,T8L
    popw    T6H,T6L
    popw    T4H,T4L
    popw    T2H,T2L
;    clr    r1
    ret
.endfunc

; Constant Tables
Log2Table:    ; Table 16*(Log2(1) thru Log2(2)), 16 values
    .dc.b   0,2,3,4,5,6,7,8,9,10,11,12,13,14,14,15

;----------------------------------------------------------------------------;
.global fft_output
.func fft_output
fft_output:
    pushw   T2H,T2L
    pushw   T4H,T4L
    pushw   T6H,T6L
    pushw   T8H,T8L
    pushw   T10H,T10L
    push    AH
    pushw   YH,YL

    movw    T10L, EL                ; T10 = array_bfly;
    movw    YL, DL                  ; Y = array_output;
    ldi     r22,0x40                ; Load R22:R23 with 64
    ldi     r23,0x00
    ldiw    ZH,ZL, tbl_bitrev       ; Z = tbl_bitrev;
    clr     EH                      ; Zero, needed in FMULS16

    ldi     AH,128                  ; FFT_N/2
    sbic    0x0008,4                ; iqfft? (bit 4 in GPIO8)
    clr     AH                      ; FFT_N

1:  lpm     XL, Z+                  ; X = *Z++; (Read index from bit reversed table)
    clr     XH
    lsl     XL                      ; X = X<<1  (Each butterfly element has 4 bytes)
    rol     XH
    lsl     XL                      ; X = X<<1
    rol     XH
    addw    XH,XL, T10H,T10L        ; X += array_bfly;
    ldw     BH,BL, X+               ; B = *X++; get real part
    ldw     CH,CL, X+               ; C = *X++; get imaginary part
    FMULS16 T4H,T4L,T2H,T2L, BH,BL, BH,BL    ;T4:T2 = B * B;
    FMULS16 T8H,T8L,T6H,T6L, CH,CL, CH,CL    ;T8:T6 = C * C;
    addd    T4H,T4L,T2H,T2L, T8H,T8L,T6H,T6L;T4:T2 += T8:T6;
    SQRT32                          ;B = sqrt(T4:T2);
    clr     r1
    sbis    0x0008,3            ; Use log? (bit 3 in GPIO8)
    rjmp    nolog               ; 2 cycles
    ; ****** Apply log to B, if it is above 64 ******
    cp      r22,BL              ; Compare with 64
    cpc     r23,BH
    brcs    sustract            ; jump if output>64
    ldi     BL,0                ; otherwise, set result as 0
    rjmp    gotit
sustract:
    sub     BL,r22
    sbc     BH,r23
    pushw   ZH,ZL
    ldiw    ZH,ZL, Log2Table
    ldi     CL, 0xF0            ; load with 240 (16*log2(2^15)=240)
    ; shift until most significant bit is set, each bit-shift results in log += 16
doshift:
    sbrc    BH,7
    rjmp    doneshift
    ROL     BL
    ROL     BH
    subi    CL,0x10             ; Substract 16
    rjmp    doshift
doneshift:
    ; BH has now form 1iii ixxx, get the next 4 bits (iiii) and address table with it
    lsl     BH                  ; remove msb (has a 1 on it)
    lsr     BH
    lsr     BH
    lsr     BH
    lsr     BH                  ; now BH is in the form 0000iiii
    add     ZL,BH               ; add offset to pointer
    adc     ZH,R1               ; R1 is zero
    lpm     CH,Z
    add     CL,CH
    mov     BH, CL
    popw    ZH,ZL
    rjmp    gotit
nolog:
    LSL     BL                  ; Condition output
    ROL     BH                  ; It seems that multiplying by four
    LSL     BL                  ; gives a good result
    ROL     BH                  ;
    BRCC    gotit
    ldi     BH, 255             ; clip if overflow
gotit:
    st Y+, BH           ; Only take high byte

    dec     AH          ; while(--A)
    rjne    1b          ;/

    popw    YH,YL
    pop     AH
    popw    T10H,T10L
    popw    T8H,T8L
    popw    T6H,T6L
    popw    T4H,T4L
    popw    T2H,T2L
;    clr    r1          ; r1 already is zero
    ret
.endfunc
