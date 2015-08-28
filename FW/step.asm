;**** Step Motor Controller
;*
;* Title:		Step Motor Controller
;* Version:		Demo 1.0
;* Target:		AT90S2313
;* CLK frequency:	10.000MHz

.include "2313def.inc"

;***** Constantes

;Initial values:
.equ	V	=2000		;speed, steps per second (VMIN..4000)
.equ	A	=2000		;acceleration, steps per squared second (1..4000)
.equ	I	=2000		;peak Iphase, mA (0..5000)

;General:
.equ	VMIN	=50			;min. speed
.equ	TBASE	=250-9		;time base, 250x0.1uS=25uS
.equ	UPDTM	=625		;speed update rate, 625x25uS=15.625mS

;***** Variables

.DSEG	;data segment (internal RAM)

TABLE:	.byte 8			;Phase table

;***** Global Register Variables

.def	temp0	=r0		;lpm data register
		
.def	STCNTL	=r1		;step program timer low byte
.def	STCNTH	=r2		;step program timer high byte

.def	URCNTL	=r3		;update rate program timer low byte
.def	URCNTH	=r4		;update rate program timer high byte

.def	TL	=r5		;step period low
.def	TH	=r6		;step period high

.def	VRL	=r7		;required speed x 64 low
.def	VRM	=r8		;required speed x 64 medium
.def	VRH	=r9		;required speed x 64 high

.def	VCL	=r10		;current speed x 64 low
.def	VCM	=r11		;current speed x 64 medium
.def	VCH	=r12		;current speed x 64 high

.def	temp	=r16		;Used in main program
.def	tempL	=r17		;Used in main program
.def	tempM	=r18		;Used in main program
.def	tempH	=r19		;Used in main program

.def	tempA	=r20		;Used in TIMER0 interrupt
.def	tempB	=r21		;Used in TIMER0 interrupt
.def	tempC	=r22		;Used in TIMER0 interrupt
.def	PHASE	=r23		;Used in TIMER0 interrupt

.def	STATE	=r24	
.equ	UPD	=0		;=STATE.0  speed update bit

.def	ResL	=r25		;Used in V2T
.def	ResM	=r26		;Used in V2T
.def	ResH	=r27		;Used in V2T

;r28,r29 used as Y register
;r30,r31 used as Z register	;Used in TIMER0 interrupt

;***** Port Definitions

;Port B:

.equ	DIRB	=0xF8		;PB7..PB3 - out
.equ	PUPB	=0x07		;PB2..PB0 - pull up on

.equ	FLM	=PB0		;Front limit
.equ	RLM	=PB1		;Rear  limit
.equ	AUXIN1	=PB2		;AUX in 1
.equ	PWMOUT	=PB3		;PWM out
.equ	PH0	=PB4		;Motor phase 0
.equ	PH1	=PB5		;Motor phase 1
.equ	PH2	=PB6		;Motor phase 2
.equ	PH3	=PB7		;Motor phase 3

;Port D:

.equ	DIRD	=0x02		;PD1 - out
.equ	PUPD	=0x7D		;PD0,PD2..PD6 - pull up on

.equ	RXD	=PD0		;RXD
.equ	TXD	=PD1		;TXD
.equ	COM0	=PD2		;Comparator 0
.equ	COM1	=PD3		;Comparator 1
.equ	FWD	=PD4		;Forward command
.equ	REW	=PD5		;Rewind  command
.equ	AUXIN2	=PD6		;AUX in 2

;***** Macros

.macro	clbr	;clear bit in register
	cbr @0,exp2(@1)
.endm

.macro	stbr	;set bit in register
	sbr @0,exp2(@1)
.endm

.macro	bbrc	;branch if bit in register clear
	sbrs @0,@1
	rjmp @2
.endm

.macro	bbrs	;branch if bit in register set
	sbrc @0,@1
	rjmp @2
.endm

.macro	bbic	;branch if bit in I/O clear
	sbis @0,@1
	rjmp @2
.endm

.macro	bbis	;branch if bit in I/O set
	sbic @0,@1
	rjmp @2
.endm

;***** Interrupt Vectors

.CSEG
.org	0
	rjmp	INIT		;Reset handle
.org	INT0addr
	rjmp	COMP0		;INT0 handle	
.org	INT1addr
	rjmp	COMP1		;INT1 handle
.org	OVF0addr
	rjmp	TIMER0		;Tiomer 0 overflow interrupt handle
	
;***** Program Execution Starts Here

INIT:	wdr			;Watchdog reset
	ldi	temp,0x0A	;Watchdog enable, 64mS
	out	WDTCR,temp
	
	ldi	temp,RAMEND	;Locate stack
	out	SPL,temp
	
;Ports setup:	
	
	ldi	temp,PUPB
	out	PORTB,temp	;Init PORTB and on/off pullup
	ldi	temp,DIRB	
	out	DDRB,temp	;Set PORTB direction
	
	ldi	temp,PUPD
	out	PORTD,temp	;Init PORTD and on/off pullup
	ldi	temp,DIRD
	out	DDRD,temp	;Set PORTD direction

;Variables init:
	
	clr	VRL		;clear required speed
	clr	VRM
	clr	VRH		
	
	clr	VCL		;clear current speed
	clr	VCM		
	clr	VCH
	
	rcall	V2T		;TL, TH init
	mov	STCNTL,TL	;load step timer
	mov	STCNTH,TH
	
	ldi	temp,low (UPDTM) ;load update timer
	mov	URCNTL,temp	
	ldi	temp,high(UPDTM)
	mov	URCNTH,temp

	clr	STATE		;clear state
	clr	PHASE		;clear PHASE
	
	rcall	LDPWM		;load PWM value
	
;	ldi	ZL,low (FSMT*2)	;load ROM full step non-interlaced table base
;	ldi	ZH,high(FSMT*2)	   
	
;	ldi	ZL,low (FSMIT*2) ;load ROM full step interlaced table base
;	ldi	ZH,high(FSMIT*2)
	
	ldi	ZL,low (HSMT*2)	;load ROM half step table base
	ldi	ZH,high(HSMT*2)
	
	ldi	temp,8		;load table size
	ldi	YL,low (TABLE)	;load RAM phase table base
	ldi	YH,high(TABLE)
	
Tb_Ld:	lpm			;read byte from ROM to temp0 (r00)
	st	Y+,temp0	;store value in RAM
	adiw	ZL,0x01		;increment flash pointer
	dec	temp		;decrement counter
	brne	Tb_Ld		;continue until table is loaded
	
;Periphery setup:

	ldi	temp,-TBASE		
	out	TCNT0,temp	;TC0 load
	ldi	temp,0x01
	out	TCCR0,temp	;CK/1 for TC0
	ldi	temp,0x02
	out	TIFR,temp	;Clear pending timer interrupt
	out	TIMSK,temp	;Enable TC0 overflow interrupt
	
	ldi	temp,0x81
	out	TCCR1A,temp	;8-bit non-inverted PWM
	ldi	temp,0x01	
	out	TCCR1B,temp	;CK/1 for TC1
	
	ldi	temp,0x00
	out	MCUCR,temp	;INT0 and INT1 level activating
	
	sei			;Enble global interrupt
	
;***** Main Loop:

MAIN:	bbis	PIND,FWD,Dir_R	;Check FORWARD command
	bbic	PIND,REW,Dir_S	;Check REWIND  command
	bbic	PINB,FLM,Dir_S	;Check FORWARD limit
	
	ldi	tempL,low (V)	;Forward: load V as required speed
	ldi	tempH,high(V)
	rcall	MUL64		;VR <- V x 64
	rjmp	Ch_Upd
	
Dir_R:	bbis	PIND,REW,Dir_S	;Check REWIND command
	bbic	PINB,RLM,Dir_S	;Check REWIND limit

	ldi	tempL,low (-V)	;load -V as required speed
	ldi	tempH,high(-V)
	rcall	MUL64		;VR <- V x 64
	rjmp	Ch_Upd

Dir_S:	clr	VRL		;Stop
	clr	VRM
	clr	VRH
	
Ch_Upd:	bbrc	STATE,UPD,No_Upd ;skip if UPD=0
	clbr	STATE,UPD	 ;clear UPD
	
	cp	VCL,VRL
	cpc	VCM,VRM
	cpc	VCH,VRH
	breq	No_Upd		;skip update if VC=VR
	brlt	Sp_Up		;speed up if VC<VR (signed)

Sp_Dn:	ldi	temp,low (A)	;VC - A
	sub	VCL,temp
	ldi	temp,high(A)
	sbc	VCM,temp
	ldi	temp,0
	sbc	VCH,temp
	
	cp	VCL,VRL
	cpc	VCM,VRM
	cpc	VCH,VRH
	brlt	V_Lim
	rjmp	Upd_T
	
Sp_Up:	ldi	temp,low (A)	;VC + A
	add	VCL,temp
	ldi	temp,high(A)
	adc	VCM,temp
	ldi	temp,0
	adc	VCH,temp
	
	cp	VCL,VRL
	cpc	VCM,VRM
	cpc	VCH,VRH
	brge	V_Lim
	rjmp	Upd_T
	
V_Lim:	mov	VCL,VRL		;VC <- VR
	mov	VCM,VRM
	mov	VCH,VRH
	
Upd_T:	rcall	V2T		;Convert VC to T	
	
No_Upd:	wdr			;Watchdog wakeup
	rjmp	MAIN
	
;***** Subroutines

;VR <- tempH,tempL x 64 (signed)

MUL64:	ser	temp
	sbrs	tempH,7
	clr	temp
	mov	VRL,tempL
	mov	VRM,tempH
	mov	VRH,temp
	ldi	temp,6
sh6:	lsl	VRL
	rol	VRM
	rol	VRH
	dec	temp
	brne	sh6
	ret

;Convert speed to step period
;VCI = |VC|
;if VCI < VMIN x 64 then VCI = VMIN * 64
;T = 2 560 000 / VCI

V2T:	push	VRL		;save VR
	push	VRM
	push	VRH
	
	ldi	tempL, low(VMIN)
	ldi	tempH,high(VMIN)
	rcall	MUL64		;VR <- VMIN x 64
	
	mov	tempL,VCL	;temp <- VC
	mov	tempM,VCM
	mov	tempH,VCH
	bbrc	tempH,7,pos	;skip if temp > 0
	subi	tempL,1		;temp = - temp
	sbci	tempM,0
	sbci	tempH,0
	com	tempL
	com	tempM
	com	tempH
	
pos:	cp	tempL,VRL
	cpc	tempM,VRM
	cpc	tempH,VRH
	brsh	mk_t
	mov	tempL,VRL
	mov	tempM,VRM
	mov	tempH,VRH
	
mk_t:	ldi	ResL,low  (2560000)
	ldi	ResM,high (2560000)
	ldi	ResH,byte3(2560000)
	
	clr	VRL		;clear remainder
	clr	VRM
	clr	VRH
	clc			;clear carry
	
	ldi	temp,25		;init loop counter
	
d24u_1:	rol	ResL		;shift left dividend
	rol	ResM
	rol	ResH
	dec	temp		;decrement counter
	brne	d24u_2		;if done
	rjmp	done		;    jump out
	
d24u_2:	rol	VRL		;shift dividend into remainder
	rol	VRM
	rol	VRH
	sub	VRL,tempL	;remainder = remainder - divisor
	sbc	VRM,tempM
	sbc	VRH,tempH	
	brcc	d24u_3		;if result negative
	add	VRL,tempL	;    restore remainder
	adc	VRM,tempM
	adc	VRH,tempH
	clc			;    clear carry to be shifted into result
	rjmp	d24u_1		;else
d24u_3:	sec			;    set carry to be shifted into result
	rjmp	d24u_1
	
done:	cli			;interrupts disable
	mov	TL,ResL		;step period load
	mov	TH,ResM
	sei			;interrupts enable
	
	pop	VRH		;restore VR
	pop	VRM
	pop	VRL
	
	ret

;Load PWM value
;OCR1A = I / 20

LDPWM:	ldi	tempL,low (I/20)
	ldi	tempH,high(I/20)
	out	OCR1AH,tempH
	out	OCR1AL,tempL
	ret

;***** Interrupts Service Routines

;INT0 Interrupt
;Current limit for phases 0 and 2

COMP0:	
	push	temp                               ;Занесение регистра в стек (просто сохраняем текущее значение temp)
	in	temp,SREG                                  ;Чтение регистра состояния SREG 
	push	temp								   ;Занесение значения регистра SREG в стек 
	in	temp,PORTB                                 ;Чтение порта
	andi	temp,0xAF                              ;Логическое И с константой 0b10101111   маска => 1010
	out	PORTB,temp	;clear phases 0 and 2
	in	temp,GIMSK
	clbr	temp,INT0
	out	GIMSK,temp	;INT0 disable
													;На выходе из прерывания вернём из стека значения SREG и temp (в обратном порядке)
	pop	temp										;загружаем из стека значение регистра SREG (в обратном порядке, тому которым записывали)
	out	SREG,temp									;загружаем значение в регистр SREG
	pop	temp										;загружаем из стека значение  temp
	reti

;INT1 Interrupt
;Current limit for phases 1 and 3

COMP1:	
	push	temp                                ;кладем текущее значение temp на стек
	in	temp, SREG                                  ;читаем регистр SREG в temp
	push	temp                                    ;кладем текущее значение temp на стек
	in	temp,PORTB
	andi	temp,0x5F                               ;Логическое И с константой 0b01011111   маска => 0101
	out	PORTB,temp	;clear phases 1 and 3
	in	temp,GIMSK
	clbr	temp,INT1
	out	GIMSK,temp	;INT1 disable
													;На выходе из прерывания вернём из стека значения SREG и temp (в обратном порядке)	
	pop	temp										;загружаем из стека в temp значение регистра SREG (в обратном порядке, тому которым записывали)
	out	SREG,temp									;загружаем значение temp в регистр SREG
	pop	temp										;загружаем из стека значение  temp
	reti

;TIMER0 Interrupt
;Uses tempA, tempB

TIMER0:	ldi	tempB,-TBASE	;reload timer0
	out	TCNT0,tempB
	in	tempA,SREG	;Save SREG
	
	ldi	tempB,1		;STCNT - 1
	sub	STCNTL,tempB
	ldi	tempB,0
	sbc	STCNTH,tempB
	brcc	NewPh		;jump if no overflow
	
	mov	STCNTL,TL	;reload step timer
	mov	STCNTH,TH
	
	mov	tempB,VCH	;Check VC
	bbrs	tempB,7,RewD	
	or	tempB,VCM
	or	tempB,VCL
	breq	NewPh
	
FwdD:	inc	PHASE		;Forward if VC>0, PHASE+1
	rjmp	NewPh
	
RewD:	dec	PHASE		;Rewind  if VC<0, PHASE-1

NewPh:	clr	tempC
	mov	tempB,VCH	;Check VC
	or	tempB,VCM
	or	tempB,VCL
	breq	Stop		;clear all phases if VC=0 (comment if you want hold mode)
	
	mov	tempB,PHASE
	andi	tempB,0x07	;table - 8 bytes
	clr	tempC
	ldi	ZL,low (TABLE)	;load table base low
	ldi	ZH,high(TABLE)	;load table base high
	add	ZL,tempB	;add offset
	adc	ZH,tempC
	ld	tempC,Z		;read table
Stop:	
	in	tempB,PORTB
	andi	tempB,0x0F
	or	tempB,tempC
	out	PORTB,tempB	;port load
	
	ldi	tempB,0xC0
	out	GIFR,tempB	;Clear pending interrupts
	out	GIMSK,tempB	;Enable INT0 and INT1 interrupts
		
	ldi	tempB,1		;URCNT - 1
	sub	URCNTL,tempB
	ldi	tempB,0
	sbc	URCNTH,tempB
	brcc	TEND		;jump if no overflow
	
	ldi	tempB,low (UPDTM)
	mov	URCNTL,tempB	;reload update timer
	ldi	tempB,high(UPDTM)
	mov	URCNTH,tempB
	stbr	STATE,UPD	;set UPD

TEND:	out	SREG,tempA	;Restore SREG
	reti
	
;***** Step tables

;Half step mode table:

HSMT:	.db 0x10,0x30,0x20,0x60,0x40,0xC0,0x80,0x90

;Full step mode interlaced table:

FSMIT:	.db 0x90,0x30,0x60,0xC0,0x90,0x30,0x60,0xC0

;Full step mode non-interlaced table:

FSMT:	.db 0x10,0x20,0x40,0x80,0x10,0x20,0x40,0x80
	
	
