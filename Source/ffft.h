#ifndef FFT_N
#define FFT_N	256		/* Number of samples (64,128,256,512). Don't forget to clean! */

#ifndef FFFT_ASM	/* for c modules */

typedef struct _tag_complex_t {
	int16_t	r;
	int16_t i;
} complex_t;

void fft_execute (complex_t *);
void fft_output (complex_t *, uint8_t *);
int8_t Sin(uint8_t angle);

#else				/* for asm module */

#define T0L	r0
#define T0H	r1
#define T2L	r2
#define T2H	r3
#define T4L	r4
#define T4H	r5
#define T6L	r6
#define T6H	r7
#define T8L	r8
#define T8H	r9
#define T10L	r10
#define T10H	r11
#define T12L	r12
#define T12H	r13
#define T14L	r14
#define T14H	r15
#define AL	r16
#define AH	r17
#define BL	r18
#define BH	r19
#define CL	r20
#define CH	r21
#define DL	r22
#define DH	r23
#define EL	r24
#define EH	r25
#define XL	r26
#define XH	r27
#define YL	r28
#define YH	r29
#define ZL	r30
#define ZH	r31

.macro	ldiw	dh,dl, abs
	ldi	\dl, lo8(\abs)
	ldi	\dh, hi8(\abs)
.endm

.macro	subiw	dh,dl, abs
	subi	\dl, lo8(\abs)
	sbci	\dh, hi8(\abs)
.endm

.macro	addiw	dh,dl, abs
	subi	\dl, lo8(\abs)
	sbci	\dh, hi8(\abs)
.endm

.macro	addw	dh,dl, sh,sl
	add	\dl, \sl
	adc	\dh, \sh
.endm

.macro	addd	d3,d2,d1,d0, s3,s2,s1,s0
	add	\d0, \s0
	adc	\d1, \s1
	adc	\d2, \s2
	adc	\d3, \s3
.endm

.macro	subw	dh,dl, sh,sl
	sub	\dl, \sl
	sbc	\dh, \sh
.endm

.macro	subd	d3,d2,d1,d0, s3,s2,s1,s0
	sub	\d0, \s0
	sbc	\d1, \s1
	sbc	\d2, \s2
	sbc	\d3, \s3
.endm

.macro	lddw	dh,dl, src
	ldd	\dl, \src
	ldd	\dh, \src+1
.endm

.macro	ldw	dh,dl, src
	ld	\dl, \src
	ld	\dh, \src
.endm

.macro	stw	dst, sh,sl
	st	\dst, \sl
	st	\dst, \sh
.endm

.macro	clrw	dh, dl
	clr	\dh
	clr	\dl
.endm

.macro	lsrw	dh, dl
	lsr	\dh
	ror	\dl
.endm

.macro	asrw	dh, dl
	asr	\dh
	ror	\dl
.endm

.macro	lslw	dh, dl
	lsl	\dl
	rol	\dh
.endm

.macro	pushw	dh, dl
	push	\dh
	push	\dl
.endm

.macro	popw	dh, dl
	pop	\dl
	pop	\dh
.endm

.macro	lpmw	dh,dl, src
	lpm	\dl, \src
	lpm	\dh, \src
.endm

.macro	rjne	lbl
	breq	99f
	rjmp	\lbl
99:
.endm


.macro	FMULS16	d3,d2,d1,d0 ,s1h,s1l, s2h,s2l	;Fractional Multiply (19clk)
	fmuls	\s1h, \s2h
	movw	\d2, T0L
	fmul	\s1l, \s2l
	movw	\d0, T0L
	adc	\d2, EH ;EH: zero reg.
	fmulsu	\s1h, \s2l
	sbc	\d3, EH
	add	\d1, T0L
	adc	\d2, T0H
	adc	\d3, EH
	fmulsu	\s2h, \s1l
	sbc	\d3, EH
	add	\d1, T0L
	adc	\d2, T0H
	adc	\d3, EH
.endm

.macro	FMULS16x8	d3,d2,d1 ,s1h,s1l, s2	;Fractional Multiply 16x8 (9clk), d0 would be 0
	fmuls	\s1h, \s2
	movw	\d2, T0L
	fmulsu	\s2, \s1l
	sbc	\d3, EH ;EH: zero reg.
	mov	\d1, T0L
	add	\d2, T0H
	adc	\d3, EH
.endm

;  r5: r4: r3:r2  - input (not modified)
;         r19:r18 - answer, floor(sqrt(input))
;         r21:r20 - trial bit, r7:r6 - trial answer, r9:r8:r27:r26 - trial^2
.macro	SQRT32          ; 32bit square root, one answer bit per pass using MUL (435clk)
    clr     r18         ; answer = 0
    clr     r19
    clr     r20         ; bit = 0x8000
    ldi     r21, 0x80
6:  movw    r6, r18     ; t = answer | bit
    or      r6, r20
    or      r7, r21
    mul     r6, r6      ; r9:r8:r27:r26 = t * t
    movw    r26, r0
    mul     r7, r7
    movw    r8, r0
    mul     r6, r7
    add     r27, r0
    adc     r8, r1
    adc     r9, EH      ; EH: zero reg.
    add     r27, r0
    adc     r8, r1
    adc     r9, EH
    cp      r2, r26     ; input >= t * t ?
    cpc     r3, r27
    cpc     r4, r8
    cpc     r5, r9
    brcs    7f
    movw    r18, r6     ; yes: answer = t
7:  lsr     r21         ; bit >>= 1
    ror     r20
    brcc    6b          ; until the bit is shifted out
.endm

#endif	/* FFFT_ASM */

#endif	/* FFT_N */
