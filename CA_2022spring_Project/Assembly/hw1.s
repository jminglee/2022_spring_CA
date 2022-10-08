.data
    n: .word 11
.text
.globl __start

FUNCTION:
    # Todo: Define your own function in HW1
    addi sp sp -8     # Move space of stack pointer
    sw x1 4(sp)       # Save current address
    sw x10 0(sp)      # Save current n
    addi x6 x0 10     # x6 = 10
    bge x10 x6 case1  # check n>=10 or not
    addi x6 x0 1      # x6 = 1
    bge x10 x6 case2  # check n>=1 or not
    addi x5 x0 7      # x5 = 7
    lw x10 0(sp)      # load prev n
    lw x1 4(sp)
    addi sp sp 8      # Move space of stack pointer
    jalr x0 0(x1)
  
case1: # T(n) = 2T(3n/4) + 0.875n - 137, n >= 10
    addi x7 x0 3      # x7 = 3
    mul x10 x10 x7    
    srli x10 x10 2    # x10 = 3*n/4
    jal x1 FUNCTION   # recursion
    lw x10 0(sp)
    lw x1 4(sp)
    addi sp sp 8      # Move space of stack pointer
    addi x7 x0 7      # x7 = 7
    mul x7 x10 x7 
    srli x7 x7 3      # x7 = 0.875*n
    addi x6 x5 0      # x6 = x5
    slli x6 x6 1      # x6 = 2*x6
    add x6 x6 x7      # x6 = x6+x7
    addi x6 x6 -137   # x6 = x5-137
    addi x5 x6 0      # x5 = x6
    jalr x0 0(x1)
  
case2: # T(n) = 2T(n-1)
    addi x10 x10 -1   # x10 = n-1
    jal x1 FUNCTION   # recursion
    lw x10 0(sp)
    lw x1 4(sp)
    addi sp sp 8      # Move space of stack pointer
    addi x6 x5 0      # x6 = x5
    slli x6 x6 1      # x6 = 2*x6
    addi x5 x6 0      # x5 = x6
    jalr x0 0(x1)

# Do NOT modify this part!!!
__start:
    la   t0, n
    lw   x10, 0(t0)
    jal  x1,FUNCTION
    la   a0, n
    sw   t0, 4(a0)
    addi a0,x0,10
    ecall