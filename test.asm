;
; Z80 test of the retro-storage library
;

; 82C55 (PPI)
;   Addresses
;     Data bus: $10
;     Data control: $12
;     PPI control: $13

org $9000
buffer:
    ds 512

org $8000
    call    iputs
    db "retro-storage test",5,0

    ; Init
    call    rsInit

    ld      hl, $100
    call    rsSetRegPtr

    ; call    iputs
    ; db "  Result: ",0
    ; call    rsReadReg

    ; ; Drive READ
    ; call    iputs
    ; db "Drive READ",5,0
    ; ld      a, 0        ; Drive_A
    ; ld      b, 1        ; READ
    ; ld      hl, $123    ; LBA
    ; call    rsDoDiskOp


    ; call    hexdump_a
    ; call    iputs
    ; db 5,0

    ; call    iputs
    ; db "Read full buffer",5,0
    ; ld      de, buffer
    ; call    rsReadFullBuffer

    jr      exit


exit:
    ld      a, $00
    ld      c, $00
    call    $0030

; ----- Helper Functions -----

rsInit:
    ; PPI mode
    ld      a, $C2
    out     ($13), a

    ; Clear register select bit
    ; ld      a, $0
    ; out     ($12), a
    ld      a, $00  ; CLEAR bit 0 of Port C
    out     ($13), a

    ret

; Write byte to MCU at current register pointer
;   Params:  A - byte to write
;   Returns: -
;   Clobbers: A
rsWriteReg:
    ; Write byte
    out     ($10), a

    ; Wait for it to be read by MCU
writeChk:
    in      a, ($12)
    and     $80
    jr      z, writeChk

    ret

; Read byte from MCU at current register pointer
;   Params:  -
;   Returns: A - Byte read
;   Clobbers: A
rsReadReg:
    ; Wait for data to be available from MCU
    in      a, ($12)
    and     $20
    jr      z, rsReadReg

    ; Get byte
    in      a, ($10)

    ret

; Set MCU's register pointer
;   Params:  HL - New 16-bit register address
;   Returns: -
rsSetRegPtr:
    ; Set register select bit (bit 0 of Port C)
    ; ld      a, $01
    ; out     ($12), a
    ld      a, $01  ; SET bit 0 of Port C
    out     ($13), a

   ld      b, 0
dly:
   djnz    dly

    ; Clear cached read data in PPI
    in      a, ($10)

    ; Wait for data to be cleared in PPI
; clearVerify:
;     in      a, ($12)
;     and     $20
;     jr      nz, clearVerify

    ; Set reg LSB
    ld      a, l
    call    rsWriteReg

    ; Set reg MSB
    ld      a, h
    call    rsWriteReg

    ; Clear register select bit
    ; ld      a, $00
    ; out     ($12), a
    ld      a, $00  ; CLEAR bit 0 of Port C
    out     ($13), a

    ; Wait for data to be set in PPI for new register
; setVerify:
;     in      a, ($12)
;     and     $20
;     jr      z, setVerify

    ret

; Read MCU's entire 512-byte R/W buffer into DE
;   Params:  DE - start address of local buffer
;   Returns: -
;   Clobbers: A, B, HL, DE
rsReadFullBuffer:
    ; Set register pointer to start of R/W buffer ($100)
    ld      hl, $100
    call    rsSetRegPtr

    ld      b, 0    ; 256
readLoop0:
    call    rsReadReg
    ld      (de), a
    inc     de
    djnz    readLoop0
    
    ld      b, 0    ; 256
readLoop1:
    call    rsReadReg
    ld      (de), a
    inc     de
    djnz    readLoop1

    ret

; Read sector on disk into MCU's buffer
;   Params:  A - Drive select (0 = Drive A, 1 = Drive B, 2 = Drive C, 3 = Drive D))
;            B - Drive operation (0 = Controller reset, 1 = Read sector from drive into buffer, 2 = Write sector to drive from buffer, 3 = Format (floppy: low-level))
;            HL - Drive LBA
;   Returns: A - Disk status
;   Clobbers: A, B, HL
rsDoDiskOp:
    ; Set register pointer to start of common parameters ($040)
    ld      hl, $40
    call    rsSetRegPtr

    ; Set drive select
    call    rsWriteReg

    ; Set drive LBA
    ld      a, l
    call    rsWriteReg
    ld      a, h
    call    rsWriteReg
    ld      a, 0
    call    rsWriteReg
    call    rsWriteReg

    ; Start drive operation
    ld      a, b
    call    rsWriteReg

    ; Get drive operation result (should wait while busy)
waitBusy:
    call    rsReadReg
    cp      5
    jr      z, waitBusy

    ret

;##############################################################
; Write the null-terminated string starting after the call
; instruction invoking this subroutine to the console.
; Clobbers AF, C
;##############################################################
iputs:
    ex      (sp),hl                 ; hl = @ of string to print
    call    puts_loop
    inc     hl                      ; point past the end of the string
    ex      (sp),hl
    ret

;##############################################################
; Write the null-terminated string starting at the address in 
; HL to the console.  
; Clobbers: AF, C
;##############################################################
puts:
    push    hl
    call    puts_loop
    pop     hl
    ret

puts_loop:
    ld      a,(hl)                  ; get the next byte to send
    or      a
    jr      z,puts_done             ; if A is zero, return
    ld      c, $02                  ; Call SCMon output character
    push    hl
    rst     $30
    pop     hl
    inc     hl                      ; point to next byte to write
    jp      puts_loop
puts_done:
    ret

;#############################################################################
; Print the value in A in hex
; Clobbers C
;#############################################################################
hexdump_a:
	push	af
	srl	a
	srl	a
	srl	a
	srl	a
	call	hexdump_nib
	pop	af
	push	af
	and	0x0f
	call	hexdump_nib
	pop	af
	ret

hexdump_nib:
	add	a, '0'
	cp	'9'+1
	jp	m,hexdump_num
	add	a, 'A'-'9'-1
hexdump_num:
    ld      c, $02                  ; Call SCMon output character
    rst     $30
