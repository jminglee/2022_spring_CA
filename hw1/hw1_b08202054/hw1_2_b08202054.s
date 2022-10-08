.globl __start

.rodata
    msg0: .string "This is HW1_2: \n"
    msg1: .string "Plaintext: "
    msg2: .string "Ciphertext: "
.text

################################################################################
# print_char function
# Usage: 
#     1. Store the beginning address in x20
#     2. Use "j print_char"
#     The function will print the string stored from x20 
#     When finish, the whole program will return value 0
print_char:
    addi a0, x0, 4
    la a1, msg2
    ecall
    
    add a1, x0, x20
    ecall
# Ends the program with status code 0
    addi a0, x0, 10
    ecall
################################################################################
################################################################################
__start:
# Prints msg
    addi a0, x0, 4
    la a1, msg0
    ecall
    
    la a1, msg1
    ecall
    
    addi a0, x0, 8
    
    li a1, 0x10130
    addi a2, x0, 2047
    ecall
# Load address of the input string into a0
    add a0, x0, a1
################################################################################   
################################################################################ 
# Write your main function here. 
# a0 stores the beginning Plaintext
# Do store 66048(0x10200) into x20 
    
li x20 0x10200
jal x1 func
j print_char

func:
  addi sp sp -8   # Move space of stack pointer
  sw x19 4(sp)    # store x19
  sw x18 0(sp)    # store x18
  add x19 x0 x0   # x19 = i = 0
  addi x18 x0 57  # x18 = comma_count = '9'
L1: # case of ','
  add x5 x19 a0   # x5 = addr of input[i]
  lbu x6 0(x5)    # x6 = input[i]
  beq x6 x0 Exit  # if input[i] == 0 then exit
  add x28 x19 x20 # x28 = addr of output[i]
  lbu x29 0(x28)  # x29 = output[i]
  addi x7 x0 44   # x7 = ','
  bne x6 x7 L2    # check input[i] is ',' or not
  add x29 x0 x18  # output[i] = comma_count
  addi x18 x18 -1 # comma_count -= 1
  j Store

L2: # case of 'a'~'m'
  addi x7 x0 110  # x6 = 'n'
  bge x6 x7 L3    # check input[i] is less than 'n'
  addi x29 x6 -19 # output[i] = input[i] - 21
  j Store
  
L3: # case of 'n'~'z'
  addi x29 x6 -45 # output[i] = input[i] - 45
  j Store

Store:
  add x20 x19 x20 # x20 = addr of output[i]
  sw x29 0(x20)   # store output[i] to addr of output[i]
  sub x20 x20 x19 # set x20 to origin value
  addi x19 x19 1  # i = i + 1
  jal x0 L1
  
Exit:
  addi x19 x19 -1
  add x20 x19 x20
  sw x0 0(x20)
  sub x20 x20 x19
  lw x19 4(sp)    # load x19
  lw x18 0(sp)    # load x18
  addi sp sp 8    # Move space of stack pointer
  jalr x0 0(x1)   # return
    
################################################################################    
