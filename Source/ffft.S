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
tbl_sin:	; Table of 127*sin(x), (0 <= x < 3pi/2, in pi/(FFT_N/2) steps); cos(x) = tbl_sin[x+FFT_N/4]
    .dc.b       0,    3,    6,    9,   12,   16,   19,   22,   25,   28,   31,   34,   37,   40,   43,   46
    .dc.b      49,   51,   54,   57,   60,   63,   65,   68,   71,   73,   76,   78,   81,   83,   85,   88
    .dc.b      90,   92,   94,   96,   98,  100,  102,  104,  106,  107,  109,  111,  112,  113,  115,  116
    .dc.b     117,  118,  120,  121,  122,  122,  123,  124,  125,  125,  126,  126,  126,  127,  127,  127
    .dc.b     127,  127,  127,  127,  126,  126,  126,  125,  125,  124,  123,  122,  122,  121,  120,  118
    .dc.b     117,  116,  115,  113,  112,  111,  109,  107,  106,  104,  102,  100,   98,   96,   94,   92
    .dc.b      90,   88,   85,   83,   81,   78,   76,   73,   71,   68,   65,   63,   60,   57,   54,   51
    .dc.b      49,   46,   43,   40,   37,   34,   31,   28,   25,   22,   19,   16,   12,    9,    6,    3
    .dc.b       0,   -3,   -6,   -9,  -12,  -16,  -19,  -22,  -25,  -28,  -31,  -34,  -37,  -40,  -43,  -46
    .dc.b     -49,  -51,  -54,  -57,  -60,  -63,  -65,  -68,  -71,  -73,  -76,  -78,  -81,  -83,  -85,  -88
    .dc.b     -90,  -92,  -94,  -96,  -98, -100, -102, -104, -106, -107, -109, -111, -112, -113, -115, -116
    .dc.b    -117, -118, -120, -121, -122, -122, -123, -124, -125, -125, -126, -126, -126, -127, -127, -127

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
    mov     r22, r24
    andi    r24, 0x7F           ; index = angle & 127
    ldiw    ZH,ZL, tbl_sin
    add     ZL, r24
    adc     ZH, r1
    lpm     r24, z
    sbrc    r22, 7              ; angle >=128
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
    movw    T12L, EL            ;/
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
    ldiw    ZH,ZL, tbl_sin      ; C = cos(T10); D = sin(T10);
    addw    ZH,ZL, T10H,T10L    ;
    lpm     DH, Z               ;
    subiw   ZH,ZL, -(FFT_N/4)   ; cos(x) = sin(x+pi/2)
    lpm     CH, Z               ;/
    movw    ZL, T0L
    FMULS16x8 T4H,T4L,T2H, AH,AL, CH    ; *Y++ = A * C + B * D;
    FMULS16x8 T8H,T8L,T6H, BH,BL, DH    ;
    add     T2H, T6H            ;
    adc     T4L, T8L            ;
    adc     T4H, T8H            ;
    stw     Y+, T4H,T4L         ;/
    FMULS16x8 T4H,T4L,T2H, BH,BL, CH    ; *Y++ = B * C - A * D;
    FMULS16x8 T8H,T8L,T6H, AH,AL, DH    ;
    sub     T2H, T6H            ;
    sbc     T4L, T8L            ;
    sbc     T4H, T8H            ;
    stw     Y+, T4H,T4L         ;/
    addw    T10H,T10L, T12H,T12L        ; T10 += T12; (next angle)
#if FFT_N >= 512
    sbrs    T10H, FFT_B - 9     ; while(T10 < pi)
#else
    sbrs    T10L, FFT_B - 1
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
;    clr    r1          ; r1 already is zero: the last multiply is "mul XH, AL" with XH = 0
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
    sbic    0x0008,4                ; iqfft? (bit 4 in GPIO8)
    rjmp    2f
    lsl     T2L                     ; Real input has .i=0: T4:T2 *= 2
    rol     T2H                     ; (same gain as the old r=i input, max 4*32512^2 < 2^32)
    rol     T4L
    rol     T4H
2:  SQRT32                          ;B = sqrt(T4:T2);
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
    BRCS    3f                  ; B >= 32768: clip
    LSL     BL                  ; gives a good result
    ROL     BH                  ;
    BRCC    gotit
3:  ldi     BH, 255             ; clip if overflow
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
