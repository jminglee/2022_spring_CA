.globl __start

.rodata
    msg0: .string "This is HW1-1: \nT(n) = 2T(3n/4) + 0.875n - 137, n >= 10\n"
    msg1: .string "T(n) = 2T(n-1), 1 <= n < 10\nT(0) = 7\n"
    msg2: .string "Enter a number: "
    msg3: .string "The result is: "
.text

__start:
  # Prints msg0
    addi a0, x0, 4
    la a1, msg0
    ecall
  # Prints msg1
    addi a0, x0, 4
    la a1, msg1
    ecall
  # Prints msg2
    addi a0, x0, 4
    la a1, msg2
    ecall
  # Reads an int
    addi a0, x0, 5
    ecall

################################################################################ 
  # Write your main function here. 
  # The input n is in a0. 
  # You should store the result T(n) into t0.
  # Round down the result of division.
  
  # HW1_1 
  # T(n) = 2T(3n/4) + 0.875n - 137, n >= 10
  # T(n) = 2T(n-1), 1 <= n < 10
  # T(0) = 7
  
jal x1 func        # Move to main function
beq x0 x0 result   # Move to result
  
func:
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
  jal x1 func       # recursion
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
  jal x1 func       # recursion
  lw x10 0(sp)
  lw x1 4(sp)
  addi sp sp 8      # Move space of stack pointer
  addi x6 x5 0      # x6 = x5
  slli x6 x6 1      # x6 = 2*x6
  addi x5 x6 0      # x5 = x6
  jalr x0 0(x1)

################################################################################

result:
  # Prints msg3
    addi a0,x0,4
    la a1, msg3
    ecall
  # Prints the result in t0
    addi a0,x0,1
    add a1,x0,t0
    ecall
  # Ends the program with status code 0
    addi a0,x0,10
    ecall    