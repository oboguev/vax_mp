        TITLE X64_UTIL

        ;;.686P

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;  Helper assembler routines for VAX MP x64 build, MSVC 64-bit compiler.
;
;  Compiled with ML64.
;  ML64 comes with Windows DDK.
;
;  Perhaps it might be also compiled (but this was not tested) with YASM, NASM or JWASM.
;  See http://en.wikibooks.org/wiki/X86_Assembly/x86_Assemblers for links.
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;+
;
;  uint64 __fastcall x64_native_adawi(uint16* pval, uint16 addend)
;
;      Performs interlockeed addition of "addendum" to "*pval".
;      Returns RFLAGS.
;
;      Inputs:
;          rcx = pval
;          dx = addend
;
;      Outputs:
;          rax = result (rflags after addition)
;
;-

PUBLIC  x64_native_adawi
_TEXT SEGMENT
        ALIGN 4
x64_native_adawi  PROC
        lock add word ptr [rcx], dx
        pushfq
        pop  rax
        ret  0
x64_native_adawi  ENDP
_TEXT ENDS

;+
;
;  t_byte __fastcall x64_cas_byte(t_byte* p, byte_t old_value, byte_t new_value)
;
;      Performs interlockeed CAS on "*p".
;      Returns old value of "*p".
;
;      Inputs:
;          rcx = p
;          dl = old_value
;          r8b = new_value
;
;      Outputs:
;          eax = result, old value of "*p"
;
;-

PUBLIC  x64_cas_byte
_TEXT SEGMENT
        ALIGN 4
x64_cas_byte  PROC
        mov   al, dl
        lock cmpxchg byte ptr [rcx], r8b
        movzx eax, al
        ret  0
x64_cas_byte  ENDP
_TEXT ENDS

END
