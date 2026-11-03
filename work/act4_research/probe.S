.text
.globl _start
_start:
 addi x1,x0,7
 mul x2,x1,x1
 csrr x3,mstatus
 fence.i
