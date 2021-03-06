??%
??
B
AddV2
x"T
y"T
z"T"
Ttype:
2	??
B
AssignVariableOp
resource
value"dtype"
dtypetype?
~
BiasAdd

value"T	
bias"T
output"T" 
Ttype:
2	"-
data_formatstringNHWC:
NHWCNCHW
8
Const
output"dtype"
valuetensor"
dtypetype
?
Conv2D

input"T
filter"T
output"T"
Ttype:	
2"
strides	list(int)"
use_cudnn_on_gpubool(",
paddingstring:
SAMEVALIDEXPLICIT""
explicit_paddings	list(int)
 "-
data_formatstringNHWC:
NHWCNCHW" 
	dilations	list(int)

?
FusedBatchNormV3
x"T

scale"U
offset"U	
mean"U
variance"U
y"T

batch_mean"U
batch_variance"U
reserve_space_1"U
reserve_space_2"U
reserve_space_3"U"
Ttype:
2"
Utype:
2"
epsilonfloat%??8"&
exponential_avg_factorfloat%  ??";
data_formatstringNHWC:
NHWCNCHWNDHWCNCDHW"
is_trainingbool(
.
Identity

input"T
output"T"	
Ttype
q
MatMul
a"T
b"T
product"T"
transpose_abool( "
transpose_bbool( "
Ttype:

2	
?
MaxPool

input"T
output"T"
Ttype0:
2	"
ksize	list(int)(0"
strides	list(int)(0",
paddingstring:
SAMEVALIDEXPLICIT""
explicit_paddings	list(int)
 ":
data_formatstringNHWC:
NHWCNCHWNCHW_VECT_C
e
MergeV2Checkpoints
checkpoint_prefixes
destination_prefix"
delete_old_dirsbool(?
=
Mul
x"T
y"T
z"T"
Ttype:
2	?

NoOp
M
Pack
values"T*N
output"T"
Nint(0"	
Ttype"
axisint 
C
Placeholder
output"dtype"
dtypetype"
shapeshape:
@
ReadVariableOp
resource
value"dtype"
dtypetype?
E
Relu
features"T
activations"T"
Ttype:
2	
[
Reshape
tensor"T
shape"Tshape
output"T"	
Ttype"
Tshapetype0:
2	
o
	RestoreV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0?
.
Rsqrt
x"T
y"T"
Ttype:

2
l
SaveV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0?
?
Select
	condition

t"T
e"T
output"T"	
Ttype
H
ShardedFilename
basename	
shard

num_shards
filename
9
Softmax
logits"T
softmax"T"
Ttype:
2
?
StatefulPartitionedCall
args2Tin
output2Tout"
Tin
list(type)("
Tout
list(type)("	
ffunc"
configstring "
config_protostring "
executor_typestring ?
@
StaticRegexFullMatch	
input

output
"
patternstring
N

StringJoin
inputs*N

output"
Nint(0"
	separatorstring 
;
Sub
x"T
y"T
z"T"
Ttype:
2	
?
VarHandleOp
resource"
	containerstring "
shared_namestring "
dtypetype"
shapeshape"#
allowed_deviceslist(string)
 ?"serve*2.4.02v2.4.0-rc4-71-g582c8d236cb8??
?
conv2d_68/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*!
shared_nameconv2d_68/kernel
}
$conv2d_68/kernel/Read/ReadVariableOpReadVariableOpconv2d_68/kernel*&
_output_shapes
:<*
dtype0
t
conv2d_68/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*
shared_nameconv2d_68/bias
m
"conv2d_68/bias/Read/ReadVariableOpReadVariableOpconv2d_68/bias*
_output_shapes
:<*
dtype0
?
batch_normalization_102/gammaVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*.
shared_namebatch_normalization_102/gamma
?
1batch_normalization_102/gamma/Read/ReadVariableOpReadVariableOpbatch_normalization_102/gamma*
_output_shapes
:<*
dtype0
?
batch_normalization_102/betaVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*-
shared_namebatch_normalization_102/beta
?
0batch_normalization_102/beta/Read/ReadVariableOpReadVariableOpbatch_normalization_102/beta*
_output_shapes
:<*
dtype0
?
#batch_normalization_102/moving_meanVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*4
shared_name%#batch_normalization_102/moving_mean
?
7batch_normalization_102/moving_mean/Read/ReadVariableOpReadVariableOp#batch_normalization_102/moving_mean*
_output_shapes
:<*
dtype0
?
'batch_normalization_102/moving_varianceVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*8
shared_name)'batch_normalization_102/moving_variance
?
;batch_normalization_102/moving_variance/Read/ReadVariableOpReadVariableOp'batch_normalization_102/moving_variance*
_output_shapes
:<*
dtype0
?
conv2d_69/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:<<*!
shared_nameconv2d_69/kernel
}
$conv2d_69/kernel/Read/ReadVariableOpReadVariableOpconv2d_69/kernel*&
_output_shapes
:<<*
dtype0
t
conv2d_69/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*
shared_nameconv2d_69/bias
m
"conv2d_69/bias/Read/ReadVariableOpReadVariableOpconv2d_69/bias*
_output_shapes
:<*
dtype0
?
batch_normalization_103/gammaVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*.
shared_namebatch_normalization_103/gamma
?
1batch_normalization_103/gamma/Read/ReadVariableOpReadVariableOpbatch_normalization_103/gamma*
_output_shapes
:<*
dtype0
?
batch_normalization_103/betaVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*-
shared_namebatch_normalization_103/beta
?
0batch_normalization_103/beta/Read/ReadVariableOpReadVariableOpbatch_normalization_103/beta*
_output_shapes
:<*
dtype0
?
#batch_normalization_103/moving_meanVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*4
shared_name%#batch_normalization_103/moving_mean
?
7batch_normalization_103/moving_mean/Read/ReadVariableOpReadVariableOp#batch_normalization_103/moving_mean*
_output_shapes
:<*
dtype0
?
'batch_normalization_103/moving_varianceVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*8
shared_name)'batch_normalization_103/moving_variance
?
;batch_normalization_103/moving_variance/Read/ReadVariableOpReadVariableOp'batch_normalization_103/moving_variance*
_output_shapes
:<*
dtype0
?
conv2d_70/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*!
shared_nameconv2d_70/kernel
}
$conv2d_70/kernel/Read/ReadVariableOpReadVariableOpconv2d_70/kernel*&
_output_shapes
:<*
dtype0
t
conv2d_70/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:*
shared_nameconv2d_70/bias
m
"conv2d_70/bias/Read/ReadVariableOpReadVariableOpconv2d_70/bias*
_output_shapes
:*
dtype0
?
batch_normalization_104/gammaVarHandleOp*
_output_shapes
: *
dtype0*
shape:*.
shared_namebatch_normalization_104/gamma
?
1batch_normalization_104/gamma/Read/ReadVariableOpReadVariableOpbatch_normalization_104/gamma*
_output_shapes
:*
dtype0
?
batch_normalization_104/betaVarHandleOp*
_output_shapes
: *
dtype0*
shape:*-
shared_namebatch_normalization_104/beta
?
0batch_normalization_104/beta/Read/ReadVariableOpReadVariableOpbatch_normalization_104/beta*
_output_shapes
:*
dtype0
?
#batch_normalization_104/moving_meanVarHandleOp*
_output_shapes
: *
dtype0*
shape:*4
shared_name%#batch_normalization_104/moving_mean
?
7batch_normalization_104/moving_mean/Read/ReadVariableOpReadVariableOp#batch_normalization_104/moving_mean*
_output_shapes
:*
dtype0
?
'batch_normalization_104/moving_varianceVarHandleOp*
_output_shapes
: *
dtype0*
shape:*8
shared_name)'batch_normalization_104/moving_variance
?
;batch_normalization_104/moving_variance/Read/ReadVariableOpReadVariableOp'batch_normalization_104/moving_variance*
_output_shapes
:*
dtype0
?
conv2d_71/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:*!
shared_nameconv2d_71/kernel
}
$conv2d_71/kernel/Read/ReadVariableOpReadVariableOpconv2d_71/kernel*&
_output_shapes
:*
dtype0
t
conv2d_71/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:*
shared_nameconv2d_71/bias
m
"conv2d_71/bias/Read/ReadVariableOpReadVariableOpconv2d_71/bias*
_output_shapes
:*
dtype0
?
batch_normalization_105/gammaVarHandleOp*
_output_shapes
: *
dtype0*
shape:*.
shared_namebatch_normalization_105/gamma
?
1batch_normalization_105/gamma/Read/ReadVariableOpReadVariableOpbatch_normalization_105/gamma*
_output_shapes
:*
dtype0
?
batch_normalization_105/betaVarHandleOp*
_output_shapes
: *
dtype0*
shape:*-
shared_namebatch_normalization_105/beta
?
0batch_normalization_105/beta/Read/ReadVariableOpReadVariableOpbatch_normalization_105/beta*
_output_shapes
:*
dtype0
?
#batch_normalization_105/moving_meanVarHandleOp*
_output_shapes
: *
dtype0*
shape:*4
shared_name%#batch_normalization_105/moving_mean
?
7batch_normalization_105/moving_mean/Read/ReadVariableOpReadVariableOp#batch_normalization_105/moving_mean*
_output_shapes
:*
dtype0
?
'batch_normalization_105/moving_varianceVarHandleOp*
_output_shapes
: *
dtype0*
shape:*8
shared_name)'batch_normalization_105/moving_variance
?
;batch_normalization_105/moving_variance/Read/ReadVariableOpReadVariableOp'batch_normalization_105/moving_variance*
_output_shapes
:*
dtype0
|
dense_51/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:
?!?* 
shared_namedense_51/kernel
u
#dense_51/kernel/Read/ReadVariableOpReadVariableOpdense_51/kernel* 
_output_shapes
:
?!?*
dtype0
s
dense_51/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*
shared_namedense_51/bias
l
!dense_51/bias/Read/ReadVariableOpReadVariableOpdense_51/bias*
_output_shapes	
:?*
dtype0
?
batch_normalization_106/gammaVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*.
shared_namebatch_normalization_106/gamma
?
1batch_normalization_106/gamma/Read/ReadVariableOpReadVariableOpbatch_normalization_106/gamma*
_output_shapes	
:?*
dtype0
?
batch_normalization_106/betaVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*-
shared_namebatch_normalization_106/beta
?
0batch_normalization_106/beta/Read/ReadVariableOpReadVariableOpbatch_normalization_106/beta*
_output_shapes	
:?*
dtype0
?
#batch_normalization_106/moving_meanVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*4
shared_name%#batch_normalization_106/moving_mean
?
7batch_normalization_106/moving_mean/Read/ReadVariableOpReadVariableOp#batch_normalization_106/moving_mean*
_output_shapes	
:?*
dtype0
?
'batch_normalization_106/moving_varianceVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*8
shared_name)'batch_normalization_106/moving_variance
?
;batch_normalization_106/moving_variance/Read/ReadVariableOpReadVariableOp'batch_normalization_106/moving_variance*
_output_shapes	
:?*
dtype0
|
dense_52/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:
??* 
shared_namedense_52/kernel
u
#dense_52/kernel/Read/ReadVariableOpReadVariableOpdense_52/kernel* 
_output_shapes
:
??*
dtype0
s
dense_52/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*
shared_namedense_52/bias
l
!dense_52/bias/Read/ReadVariableOpReadVariableOpdense_52/bias*
_output_shapes	
:?*
dtype0
?
batch_normalization_107/gammaVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*.
shared_namebatch_normalization_107/gamma
?
1batch_normalization_107/gamma/Read/ReadVariableOpReadVariableOpbatch_normalization_107/gamma*
_output_shapes	
:?*
dtype0
?
batch_normalization_107/betaVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*-
shared_namebatch_normalization_107/beta
?
0batch_normalization_107/beta/Read/ReadVariableOpReadVariableOpbatch_normalization_107/beta*
_output_shapes	
:?*
dtype0
?
#batch_normalization_107/moving_meanVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*4
shared_name%#batch_normalization_107/moving_mean
?
7batch_normalization_107/moving_mean/Read/ReadVariableOpReadVariableOp#batch_normalization_107/moving_mean*
_output_shapes	
:?*
dtype0
?
'batch_normalization_107/moving_varianceVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*8
shared_name)'batch_normalization_107/moving_variance
?
;batch_normalization_107/moving_variance/Read/ReadVariableOpReadVariableOp'batch_normalization_107/moving_variance*
_output_shapes	
:?*
dtype0
{
dense_53/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:	?* 
shared_namedense_53/kernel
t
#dense_53/kernel/Read/ReadVariableOpReadVariableOpdense_53/kernel*
_output_shapes
:	?*
dtype0
r
dense_53/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:*
shared_namedense_53/bias
k
!dense_53/bias/Read/ReadVariableOpReadVariableOpdense_53/bias*
_output_shapes
:*
dtype0
f
	Adam/iterVarHandleOp*
_output_shapes
: *
dtype0	*
shape: *
shared_name	Adam/iter
_
Adam/iter/Read/ReadVariableOpReadVariableOp	Adam/iter*
_output_shapes
: *
dtype0	
j
Adam/beta_1VarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_nameAdam/beta_1
c
Adam/beta_1/Read/ReadVariableOpReadVariableOpAdam/beta_1*
_output_shapes
: *
dtype0
j
Adam/beta_2VarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_nameAdam/beta_2
c
Adam/beta_2/Read/ReadVariableOpReadVariableOpAdam/beta_2*
_output_shapes
: *
dtype0
h

Adam/decayVarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_name
Adam/decay
a
Adam/decay/Read/ReadVariableOpReadVariableOp
Adam/decay*
_output_shapes
: *
dtype0
x
Adam/learning_rateVarHandleOp*
_output_shapes
: *
dtype0*
shape: *#
shared_nameAdam/learning_rate
q
&Adam/learning_rate/Read/ReadVariableOpReadVariableOpAdam/learning_rate*
_output_shapes
: *
dtype0
^
totalVarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_nametotal
W
total/Read/ReadVariableOpReadVariableOptotal*
_output_shapes
: *
dtype0
^
countVarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_namecount
W
count/Read/ReadVariableOpReadVariableOpcount*
_output_shapes
: *
dtype0
b
total_1VarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_name	total_1
[
total_1/Read/ReadVariableOpReadVariableOptotal_1*
_output_shapes
: *
dtype0
b
count_1VarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_name	count_1
[
count_1/Read/ReadVariableOpReadVariableOpcount_1*
_output_shapes
: *
dtype0
?
Adam/conv2d_68/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*(
shared_nameAdam/conv2d_68/kernel/m
?
+Adam/conv2d_68/kernel/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_68/kernel/m*&
_output_shapes
:<*
dtype0
?
Adam/conv2d_68/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*&
shared_nameAdam/conv2d_68/bias/m
{
)Adam/conv2d_68/bias/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_68/bias/m*
_output_shapes
:<*
dtype0
?
$Adam/batch_normalization_102/gamma/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*5
shared_name&$Adam/batch_normalization_102/gamma/m
?
8Adam/batch_normalization_102/gamma/m/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_102/gamma/m*
_output_shapes
:<*
dtype0
?
#Adam/batch_normalization_102/beta/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*4
shared_name%#Adam/batch_normalization_102/beta/m
?
7Adam/batch_normalization_102/beta/m/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_102/beta/m*
_output_shapes
:<*
dtype0
?
Adam/conv2d_69/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<<*(
shared_nameAdam/conv2d_69/kernel/m
?
+Adam/conv2d_69/kernel/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_69/kernel/m*&
_output_shapes
:<<*
dtype0
?
Adam/conv2d_69/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*&
shared_nameAdam/conv2d_69/bias/m
{
)Adam/conv2d_69/bias/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_69/bias/m*
_output_shapes
:<*
dtype0
?
$Adam/batch_normalization_103/gamma/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*5
shared_name&$Adam/batch_normalization_103/gamma/m
?
8Adam/batch_normalization_103/gamma/m/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_103/gamma/m*
_output_shapes
:<*
dtype0
?
#Adam/batch_normalization_103/beta/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*4
shared_name%#Adam/batch_normalization_103/beta/m
?
7Adam/batch_normalization_103/beta/m/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_103/beta/m*
_output_shapes
:<*
dtype0
?
Adam/conv2d_70/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*(
shared_nameAdam/conv2d_70/kernel/m
?
+Adam/conv2d_70/kernel/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_70/kernel/m*&
_output_shapes
:<*
dtype0
?
Adam/conv2d_70/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*&
shared_nameAdam/conv2d_70/bias/m
{
)Adam/conv2d_70/bias/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_70/bias/m*
_output_shapes
:*
dtype0
?
$Adam/batch_normalization_104/gamma/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*5
shared_name&$Adam/batch_normalization_104/gamma/m
?
8Adam/batch_normalization_104/gamma/m/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_104/gamma/m*
_output_shapes
:*
dtype0
?
#Adam/batch_normalization_104/beta/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*4
shared_name%#Adam/batch_normalization_104/beta/m
?
7Adam/batch_normalization_104/beta/m/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_104/beta/m*
_output_shapes
:*
dtype0
?
Adam/conv2d_71/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*(
shared_nameAdam/conv2d_71/kernel/m
?
+Adam/conv2d_71/kernel/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_71/kernel/m*&
_output_shapes
:*
dtype0
?
Adam/conv2d_71/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*&
shared_nameAdam/conv2d_71/bias/m
{
)Adam/conv2d_71/bias/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_71/bias/m*
_output_shapes
:*
dtype0
?
$Adam/batch_normalization_105/gamma/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*5
shared_name&$Adam/batch_normalization_105/gamma/m
?
8Adam/batch_normalization_105/gamma/m/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_105/gamma/m*
_output_shapes
:*
dtype0
?
#Adam/batch_normalization_105/beta/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*4
shared_name%#Adam/batch_normalization_105/beta/m
?
7Adam/batch_normalization_105/beta/m/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_105/beta/m*
_output_shapes
:*
dtype0
?
Adam/dense_51/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:
?!?*'
shared_nameAdam/dense_51/kernel/m
?
*Adam/dense_51/kernel/m/Read/ReadVariableOpReadVariableOpAdam/dense_51/kernel/m* 
_output_shapes
:
?!?*
dtype0
?
Adam/dense_51/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*%
shared_nameAdam/dense_51/bias/m
z
(Adam/dense_51/bias/m/Read/ReadVariableOpReadVariableOpAdam/dense_51/bias/m*
_output_shapes	
:?*
dtype0
?
$Adam/batch_normalization_106/gamma/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*5
shared_name&$Adam/batch_normalization_106/gamma/m
?
8Adam/batch_normalization_106/gamma/m/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_106/gamma/m*
_output_shapes	
:?*
dtype0
?
#Adam/batch_normalization_106/beta/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*4
shared_name%#Adam/batch_normalization_106/beta/m
?
7Adam/batch_normalization_106/beta/m/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_106/beta/m*
_output_shapes	
:?*
dtype0
?
Adam/dense_52/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:
??*'
shared_nameAdam/dense_52/kernel/m
?
*Adam/dense_52/kernel/m/Read/ReadVariableOpReadVariableOpAdam/dense_52/kernel/m* 
_output_shapes
:
??*
dtype0
?
Adam/dense_52/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*%
shared_nameAdam/dense_52/bias/m
z
(Adam/dense_52/bias/m/Read/ReadVariableOpReadVariableOpAdam/dense_52/bias/m*
_output_shapes	
:?*
dtype0
?
$Adam/batch_normalization_107/gamma/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*5
shared_name&$Adam/batch_normalization_107/gamma/m
?
8Adam/batch_normalization_107/gamma/m/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_107/gamma/m*
_output_shapes	
:?*
dtype0
?
#Adam/batch_normalization_107/beta/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*4
shared_name%#Adam/batch_normalization_107/beta/m
?
7Adam/batch_normalization_107/beta/m/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_107/beta/m*
_output_shapes	
:?*
dtype0
?
Adam/dense_53/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:	?*'
shared_nameAdam/dense_53/kernel/m
?
*Adam/dense_53/kernel/m/Read/ReadVariableOpReadVariableOpAdam/dense_53/kernel/m*
_output_shapes
:	?*
dtype0
?
Adam/dense_53/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*%
shared_nameAdam/dense_53/bias/m
y
(Adam/dense_53/bias/m/Read/ReadVariableOpReadVariableOpAdam/dense_53/bias/m*
_output_shapes
:*
dtype0
?
Adam/conv2d_68/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*(
shared_nameAdam/conv2d_68/kernel/v
?
+Adam/conv2d_68/kernel/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_68/kernel/v*&
_output_shapes
:<*
dtype0
?
Adam/conv2d_68/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*&
shared_nameAdam/conv2d_68/bias/v
{
)Adam/conv2d_68/bias/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_68/bias/v*
_output_shapes
:<*
dtype0
?
$Adam/batch_normalization_102/gamma/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*5
shared_name&$Adam/batch_normalization_102/gamma/v
?
8Adam/batch_normalization_102/gamma/v/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_102/gamma/v*
_output_shapes
:<*
dtype0
?
#Adam/batch_normalization_102/beta/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*4
shared_name%#Adam/batch_normalization_102/beta/v
?
7Adam/batch_normalization_102/beta/v/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_102/beta/v*
_output_shapes
:<*
dtype0
?
Adam/conv2d_69/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<<*(
shared_nameAdam/conv2d_69/kernel/v
?
+Adam/conv2d_69/kernel/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_69/kernel/v*&
_output_shapes
:<<*
dtype0
?
Adam/conv2d_69/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*&
shared_nameAdam/conv2d_69/bias/v
{
)Adam/conv2d_69/bias/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_69/bias/v*
_output_shapes
:<*
dtype0
?
$Adam/batch_normalization_103/gamma/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*5
shared_name&$Adam/batch_normalization_103/gamma/v
?
8Adam/batch_normalization_103/gamma/v/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_103/gamma/v*
_output_shapes
:<*
dtype0
?
#Adam/batch_normalization_103/beta/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*4
shared_name%#Adam/batch_normalization_103/beta/v
?
7Adam/batch_normalization_103/beta/v/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_103/beta/v*
_output_shapes
:<*
dtype0
?
Adam/conv2d_70/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*(
shared_nameAdam/conv2d_70/kernel/v
?
+Adam/conv2d_70/kernel/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_70/kernel/v*&
_output_shapes
:<*
dtype0
?
Adam/conv2d_70/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*&
shared_nameAdam/conv2d_70/bias/v
{
)Adam/conv2d_70/bias/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_70/bias/v*
_output_shapes
:*
dtype0
?
$Adam/batch_normalization_104/gamma/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*5
shared_name&$Adam/batch_normalization_104/gamma/v
?
8Adam/batch_normalization_104/gamma/v/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_104/gamma/v*
_output_shapes
:*
dtype0
?
#Adam/batch_normalization_104/beta/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*4
shared_name%#Adam/batch_normalization_104/beta/v
?
7Adam/batch_normalization_104/beta/v/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_104/beta/v*
_output_shapes
:*
dtype0
?
Adam/conv2d_71/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*(
shared_nameAdam/conv2d_71/kernel/v
?
+Adam/conv2d_71/kernel/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_71/kernel/v*&
_output_shapes
:*
dtype0
?
Adam/conv2d_71/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*&
shared_nameAdam/conv2d_71/bias/v
{
)Adam/conv2d_71/bias/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_71/bias/v*
_output_shapes
:*
dtype0
?
$Adam/batch_normalization_105/gamma/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*5
shared_name&$Adam/batch_normalization_105/gamma/v
?
8Adam/batch_normalization_105/gamma/v/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_105/gamma/v*
_output_shapes
:*
dtype0
?
#Adam/batch_normalization_105/beta/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*4
shared_name%#Adam/batch_normalization_105/beta/v
?
7Adam/batch_normalization_105/beta/v/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_105/beta/v*
_output_shapes
:*
dtype0
?
Adam/dense_51/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:
?!?*'
shared_nameAdam/dense_51/kernel/v
?
*Adam/dense_51/kernel/v/Read/ReadVariableOpReadVariableOpAdam/dense_51/kernel/v* 
_output_shapes
:
?!?*
dtype0
?
Adam/dense_51/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*%
shared_nameAdam/dense_51/bias/v
z
(Adam/dense_51/bias/v/Read/ReadVariableOpReadVariableOpAdam/dense_51/bias/v*
_output_shapes	
:?*
dtype0
?
$Adam/batch_normalization_106/gamma/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*5
shared_name&$Adam/batch_normalization_106/gamma/v
?
8Adam/batch_normalization_106/gamma/v/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_106/gamma/v*
_output_shapes	
:?*
dtype0
?
#Adam/batch_normalization_106/beta/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*4
shared_name%#Adam/batch_normalization_106/beta/v
?
7Adam/batch_normalization_106/beta/v/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_106/beta/v*
_output_shapes	
:?*
dtype0
?
Adam/dense_52/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:
??*'
shared_nameAdam/dense_52/kernel/v
?
*Adam/dense_52/kernel/v/Read/ReadVariableOpReadVariableOpAdam/dense_52/kernel/v* 
_output_shapes
:
??*
dtype0
?
Adam/dense_52/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*%
shared_nameAdam/dense_52/bias/v
z
(Adam/dense_52/bias/v/Read/ReadVariableOpReadVariableOpAdam/dense_52/bias/v*
_output_shapes	
:?*
dtype0
?
$Adam/batch_normalization_107/gamma/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*5
shared_name&$Adam/batch_normalization_107/gamma/v
?
8Adam/batch_normalization_107/gamma/v/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_107/gamma/v*
_output_shapes	
:?*
dtype0
?
#Adam/batch_normalization_107/beta/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*4
shared_name%#Adam/batch_normalization_107/beta/v
?
7Adam/batch_normalization_107/beta/v/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_107/beta/v*
_output_shapes	
:?*
dtype0
?
Adam/dense_53/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:	?*'
shared_nameAdam/dense_53/kernel/v
?
*Adam/dense_53/kernel/v/Read/ReadVariableOpReadVariableOpAdam/dense_53/kernel/v*
_output_shapes
:	?*
dtype0
?
Adam/dense_53/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*%
shared_nameAdam/dense_53/bias/v
y
(Adam/dense_53/bias/v/Read/ReadVariableOpReadVariableOpAdam/dense_53/bias/v*
_output_shapes
:*
dtype0

NoOpNoOp
??
ConstConst"/device:CPU:0*
_output_shapes
: *
dtype0*??
value??B?? B??
?
layer_with_weights-0
layer-0
layer-1
layer_with_weights-1
layer-2
layer_with_weights-2
layer-3
layer-4
layer_with_weights-3
layer-5
layer-6
layer_with_weights-4
layer-7
	layer-8

layer_with_weights-5

layer-9
layer_with_weights-6
layer-10
layer-11
layer_with_weights-7
layer-12
layer-13
layer-14
layer-15
layer_with_weights-8
layer-16
layer-17
layer_with_weights-9
layer-18
layer-19
layer-20
layer_with_weights-10
layer-21
layer-22
layer_with_weights-11
layer-23
layer-24
layer_with_weights-12
layer-25
layer-26
	optimizer
	variables
regularization_losses
trainable_variables
 	keras_api
!
signatures
h

"kernel
#bias
$	variables
%regularization_losses
&trainable_variables
'	keras_api
R
(	variables
)regularization_losses
*trainable_variables
+	keras_api
?
,axis
	-gamma
.beta
/moving_mean
0moving_variance
1	variables
2regularization_losses
3trainable_variables
4	keras_api
h

5kernel
6bias
7	variables
8regularization_losses
9trainable_variables
:	keras_api
R
;	variables
<regularization_losses
=trainable_variables
>	keras_api
?
?axis
	@gamma
Abeta
Bmoving_mean
Cmoving_variance
D	variables
Eregularization_losses
Ftrainable_variables
G	keras_api
R
H	variables
Iregularization_losses
Jtrainable_variables
K	keras_api
h

Lkernel
Mbias
N	variables
Oregularization_losses
Ptrainable_variables
Q	keras_api
R
R	variables
Sregularization_losses
Ttrainable_variables
U	keras_api
?
Vaxis
	Wgamma
Xbeta
Ymoving_mean
Zmoving_variance
[	variables
\regularization_losses
]trainable_variables
^	keras_api
h

_kernel
`bias
a	variables
bregularization_losses
ctrainable_variables
d	keras_api
R
e	variables
fregularization_losses
gtrainable_variables
h	keras_api
?
iaxis
	jgamma
kbeta
lmoving_mean
mmoving_variance
n	variables
oregularization_losses
ptrainable_variables
q	keras_api
R
r	variables
sregularization_losses
ttrainable_variables
u	keras_api
R
v	variables
wregularization_losses
xtrainable_variables
y	keras_api
R
z	variables
{regularization_losses
|trainable_variables
}	keras_api
l

~kernel
bias
?	variables
?regularization_losses
?trainable_variables
?	keras_api
V
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?
	?axis

?gamma
	?beta
?moving_mean
?moving_variance
?	variables
?regularization_losses
?trainable_variables
?	keras_api
V
?	variables
?regularization_losses
?trainable_variables
?	keras_api
V
?	variables
?regularization_losses
?trainable_variables
?	keras_api
n
?kernel
	?bias
?	variables
?regularization_losses
?trainable_variables
?	keras_api
V
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?
	?axis

?gamma
	?beta
?moving_mean
?moving_variance
?	variables
?regularization_losses
?trainable_variables
?	keras_api
V
?	variables
?regularization_losses
?trainable_variables
?	keras_api
n
?kernel
	?bias
?	variables
?regularization_losses
?trainable_variables
?	keras_api
V
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?
	?iter
?beta_1
?beta_2

?decay
?learning_rate"m?#m?-m?.m?5m?6m?@m?Am?Lm?Mm?Wm?Xm?_m?`m?jm?km?~m?m?	?m?	?m?	?m?	?m?	?m?	?m?	?m?	?m?"v?#v?-v?.v?5v?6v?@v?Av?Lv?Mv?Wv?Xv?_v?`v?jv?kv?~v?v?	?v?	?v?	?v?	?v?	?v?	?v?	?v?	?v?
?
"0
#1
-2
.3
/4
05
56
67
@8
A9
B10
C11
L12
M13
W14
X15
Y16
Z17
_18
`19
j20
k21
l22
m23
~24
25
?26
?27
?28
?29
?30
?31
?32
?33
?34
?35
?36
?37
 
?
"0
#1
-2
.3
54
65
@6
A7
L8
M9
W10
X11
_12
`13
j14
k15
~16
17
?18
?19
?20
?21
?22
?23
?24
?25
?
?metrics
	variables
?non_trainable_variables
regularization_losses
trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
\Z
VARIABLE_VALUEconv2d_68/kernel6layer_with_weights-0/kernel/.ATTRIBUTES/VARIABLE_VALUE
XV
VARIABLE_VALUEconv2d_68/bias4layer_with_weights-0/bias/.ATTRIBUTES/VARIABLE_VALUE

"0
#1
 

"0
#1
?
?metrics
$	variables
?non_trainable_variables
%regularization_losses
&trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
 
 
?
?metrics
(	variables
?non_trainable_variables
)regularization_losses
*trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
hf
VARIABLE_VALUEbatch_normalization_102/gamma5layer_with_weights-1/gamma/.ATTRIBUTES/VARIABLE_VALUE
fd
VARIABLE_VALUEbatch_normalization_102/beta4layer_with_weights-1/beta/.ATTRIBUTES/VARIABLE_VALUE
tr
VARIABLE_VALUE#batch_normalization_102/moving_mean;layer_with_weights-1/moving_mean/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUE'batch_normalization_102/moving_variance?layer_with_weights-1/moving_variance/.ATTRIBUTES/VARIABLE_VALUE

-0
.1
/2
03
 

-0
.1
?
?metrics
1	variables
?non_trainable_variables
2regularization_losses
3trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
\Z
VARIABLE_VALUEconv2d_69/kernel6layer_with_weights-2/kernel/.ATTRIBUTES/VARIABLE_VALUE
XV
VARIABLE_VALUEconv2d_69/bias4layer_with_weights-2/bias/.ATTRIBUTES/VARIABLE_VALUE

50
61
 

50
61
?
?metrics
7	variables
?non_trainable_variables
8regularization_losses
9trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
 
 
?
?metrics
;	variables
?non_trainable_variables
<regularization_losses
=trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
hf
VARIABLE_VALUEbatch_normalization_103/gamma5layer_with_weights-3/gamma/.ATTRIBUTES/VARIABLE_VALUE
fd
VARIABLE_VALUEbatch_normalization_103/beta4layer_with_weights-3/beta/.ATTRIBUTES/VARIABLE_VALUE
tr
VARIABLE_VALUE#batch_normalization_103/moving_mean;layer_with_weights-3/moving_mean/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUE'batch_normalization_103/moving_variance?layer_with_weights-3/moving_variance/.ATTRIBUTES/VARIABLE_VALUE

@0
A1
B2
C3
 

@0
A1
?
?metrics
D	variables
?non_trainable_variables
Eregularization_losses
Ftrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
 
 
?
?metrics
H	variables
?non_trainable_variables
Iregularization_losses
Jtrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
\Z
VARIABLE_VALUEconv2d_70/kernel6layer_with_weights-4/kernel/.ATTRIBUTES/VARIABLE_VALUE
XV
VARIABLE_VALUEconv2d_70/bias4layer_with_weights-4/bias/.ATTRIBUTES/VARIABLE_VALUE

L0
M1
 

L0
M1
?
?metrics
N	variables
?non_trainable_variables
Oregularization_losses
Ptrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
 
 
?
?metrics
R	variables
?non_trainable_variables
Sregularization_losses
Ttrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
hf
VARIABLE_VALUEbatch_normalization_104/gamma5layer_with_weights-5/gamma/.ATTRIBUTES/VARIABLE_VALUE
fd
VARIABLE_VALUEbatch_normalization_104/beta4layer_with_weights-5/beta/.ATTRIBUTES/VARIABLE_VALUE
tr
VARIABLE_VALUE#batch_normalization_104/moving_mean;layer_with_weights-5/moving_mean/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUE'batch_normalization_104/moving_variance?layer_with_weights-5/moving_variance/.ATTRIBUTES/VARIABLE_VALUE

W0
X1
Y2
Z3
 

W0
X1
?
?metrics
[	variables
?non_trainable_variables
\regularization_losses
]trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
\Z
VARIABLE_VALUEconv2d_71/kernel6layer_with_weights-6/kernel/.ATTRIBUTES/VARIABLE_VALUE
XV
VARIABLE_VALUEconv2d_71/bias4layer_with_weights-6/bias/.ATTRIBUTES/VARIABLE_VALUE

_0
`1
 

_0
`1
?
?metrics
a	variables
?non_trainable_variables
bregularization_losses
ctrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
 
 
?
?metrics
e	variables
?non_trainable_variables
fregularization_losses
gtrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
hf
VARIABLE_VALUEbatch_normalization_105/gamma5layer_with_weights-7/gamma/.ATTRIBUTES/VARIABLE_VALUE
fd
VARIABLE_VALUEbatch_normalization_105/beta4layer_with_weights-7/beta/.ATTRIBUTES/VARIABLE_VALUE
tr
VARIABLE_VALUE#batch_normalization_105/moving_mean;layer_with_weights-7/moving_mean/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUE'batch_normalization_105/moving_variance?layer_with_weights-7/moving_variance/.ATTRIBUTES/VARIABLE_VALUE

j0
k1
l2
m3
 

j0
k1
?
?metrics
n	variables
?non_trainable_variables
oregularization_losses
ptrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
 
 
?
?metrics
r	variables
?non_trainable_variables
sregularization_losses
ttrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
 
 
?
?metrics
v	variables
?non_trainable_variables
wregularization_losses
xtrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
 
 
?
?metrics
z	variables
?non_trainable_variables
{regularization_losses
|trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
[Y
VARIABLE_VALUEdense_51/kernel6layer_with_weights-8/kernel/.ATTRIBUTES/VARIABLE_VALUE
WU
VARIABLE_VALUEdense_51/bias4layer_with_weights-8/bias/.ATTRIBUTES/VARIABLE_VALUE

~0
1
 

~0
1
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
 
 
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
hf
VARIABLE_VALUEbatch_normalization_106/gamma5layer_with_weights-9/gamma/.ATTRIBUTES/VARIABLE_VALUE
fd
VARIABLE_VALUEbatch_normalization_106/beta4layer_with_weights-9/beta/.ATTRIBUTES/VARIABLE_VALUE
tr
VARIABLE_VALUE#batch_normalization_106/moving_mean;layer_with_weights-9/moving_mean/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUE'batch_normalization_106/moving_variance?layer_with_weights-9/moving_variance/.ATTRIBUTES/VARIABLE_VALUE
 
?0
?1
?2
?3
 

?0
?1
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
 
 
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
 
 
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
\Z
VARIABLE_VALUEdense_52/kernel7layer_with_weights-10/kernel/.ATTRIBUTES/VARIABLE_VALUE
XV
VARIABLE_VALUEdense_52/bias5layer_with_weights-10/bias/.ATTRIBUTES/VARIABLE_VALUE

?0
?1
 

?0
?1
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
 
 
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
ig
VARIABLE_VALUEbatch_normalization_107/gamma6layer_with_weights-11/gamma/.ATTRIBUTES/VARIABLE_VALUE
ge
VARIABLE_VALUEbatch_normalization_107/beta5layer_with_weights-11/beta/.ATTRIBUTES/VARIABLE_VALUE
us
VARIABLE_VALUE#batch_normalization_107/moving_mean<layer_with_weights-11/moving_mean/.ATTRIBUTES/VARIABLE_VALUE
}{
VARIABLE_VALUE'batch_normalization_107/moving_variance@layer_with_weights-11/moving_variance/.ATTRIBUTES/VARIABLE_VALUE
 
?0
?1
?2
?3
 

?0
?1
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
 
 
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
\Z
VARIABLE_VALUEdense_53/kernel7layer_with_weights-12/kernel/.ATTRIBUTES/VARIABLE_VALUE
XV
VARIABLE_VALUEdense_53/bias5layer_with_weights-12/bias/.ATTRIBUTES/VARIABLE_VALUE

?0
?1
 

?0
?1
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
 
 
 
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
HF
VARIABLE_VALUE	Adam/iter)optimizer/iter/.ATTRIBUTES/VARIABLE_VALUE
LJ
VARIABLE_VALUEAdam/beta_1+optimizer/beta_1/.ATTRIBUTES/VARIABLE_VALUE
LJ
VARIABLE_VALUEAdam/beta_2+optimizer/beta_2/.ATTRIBUTES/VARIABLE_VALUE
JH
VARIABLE_VALUE
Adam/decay*optimizer/decay/.ATTRIBUTES/VARIABLE_VALUE
ZX
VARIABLE_VALUEAdam/learning_rate2optimizer/learning_rate/.ATTRIBUTES/VARIABLE_VALUE

?0
?1
Z
/0
01
B2
C3
Y4
Z5
l6
m7
?8
?9
?10
?11
?
0
1
2
3
4
5
6
7
	8

9
10
11
12
13
14
15
16
17
18
19
20
21
22
23
24
25
26
 
 
 
 
 
 
 
 
 
 
 
 
 

/0
01
 
 
 
 
 
 
 
 
 
 
 
 
 
 

B0
C1
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

Y0
Z1
 
 
 
 
 
 
 
 
 
 
 
 
 
 

l0
m1
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

?0
?1
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

?0
?1
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
8

?total

?count
?	variables
?	keras_api
I

?total

?count
?
_fn_kwargs
?	variables
?	keras_api
OM
VARIABLE_VALUEtotal4keras_api/metrics/0/total/.ATTRIBUTES/VARIABLE_VALUE
OM
VARIABLE_VALUEcount4keras_api/metrics/0/count/.ATTRIBUTES/VARIABLE_VALUE

?0
?1

?	variables
QO
VARIABLE_VALUEtotal_14keras_api/metrics/1/total/.ATTRIBUTES/VARIABLE_VALUE
QO
VARIABLE_VALUEcount_14keras_api/metrics/1/count/.ATTRIBUTES/VARIABLE_VALUE
 

?0
?1

?	variables
}
VARIABLE_VALUEAdam/conv2d_68/kernel/mRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv2d_68/bias/mPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_102/gamma/mQlayer_with_weights-1/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_102/beta/mPlayer_with_weights-1/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/conv2d_69/kernel/mRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv2d_69/bias/mPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_103/gamma/mQlayer_with_weights-3/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_103/beta/mPlayer_with_weights-3/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/conv2d_70/kernel/mRlayer_with_weights-4/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv2d_70/bias/mPlayer_with_weights-4/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_104/gamma/mQlayer_with_weights-5/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_104/beta/mPlayer_with_weights-5/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/conv2d_71/kernel/mRlayer_with_weights-6/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv2d_71/bias/mPlayer_with_weights-6/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_105/gamma/mQlayer_with_weights-7/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_105/beta/mPlayer_with_weights-7/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
~|
VARIABLE_VALUEAdam/dense_51/kernel/mRlayer_with_weights-8/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
zx
VARIABLE_VALUEAdam/dense_51/bias/mPlayer_with_weights-8/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_106/gamma/mQlayer_with_weights-9/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_106/beta/mPlayer_with_weights-9/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/dense_52/kernel/mSlayer_with_weights-10/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/dense_52/bias/mQlayer_with_weights-10/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_107/gamma/mRlayer_with_weights-11/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_107/beta/mQlayer_with_weights-11/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/dense_53/kernel/mSlayer_with_weights-12/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/dense_53/bias/mQlayer_with_weights-12/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/conv2d_68/kernel/vRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv2d_68/bias/vPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_102/gamma/vQlayer_with_weights-1/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_102/beta/vPlayer_with_weights-1/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/conv2d_69/kernel/vRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv2d_69/bias/vPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_103/gamma/vQlayer_with_weights-3/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_103/beta/vPlayer_with_weights-3/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/conv2d_70/kernel/vRlayer_with_weights-4/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv2d_70/bias/vPlayer_with_weights-4/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_104/gamma/vQlayer_with_weights-5/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_104/beta/vPlayer_with_weights-5/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/conv2d_71/kernel/vRlayer_with_weights-6/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv2d_71/bias/vPlayer_with_weights-6/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_105/gamma/vQlayer_with_weights-7/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_105/beta/vPlayer_with_weights-7/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
~|
VARIABLE_VALUEAdam/dense_51/kernel/vRlayer_with_weights-8/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
zx
VARIABLE_VALUEAdam/dense_51/bias/vPlayer_with_weights-8/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_106/gamma/vQlayer_with_weights-9/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_106/beta/vPlayer_with_weights-9/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/dense_52/kernel/vSlayer_with_weights-10/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/dense_52/bias/vQlayer_with_weights-10/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_107/gamma/vRlayer_with_weights-11/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_107/beta/vQlayer_with_weights-11/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/dense_53/kernel/vSlayer_with_weights-12/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/dense_53/bias/vQlayer_with_weights-12/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
?
serving_default_conv2d_68_inputPlaceholder*/
_output_shapes
:?????????@@*
dtype0*$
shape:?????????@@
?
StatefulPartitionedCallStatefulPartitionedCallserving_default_conv2d_68_inputconv2d_68/kernelconv2d_68/biasbatch_normalization_102/gammabatch_normalization_102/beta#batch_normalization_102/moving_mean'batch_normalization_102/moving_varianceconv2d_69/kernelconv2d_69/biasbatch_normalization_103/gammabatch_normalization_103/beta#batch_normalization_103/moving_mean'batch_normalization_103/moving_varianceconv2d_70/kernelconv2d_70/biasbatch_normalization_104/gammabatch_normalization_104/beta#batch_normalization_104/moving_mean'batch_normalization_104/moving_varianceconv2d_71/kernelconv2d_71/biasbatch_normalization_105/gammabatch_normalization_105/beta#batch_normalization_105/moving_mean'batch_normalization_105/moving_variancedense_51/kerneldense_51/bias'batch_normalization_106/moving_variancebatch_normalization_106/gamma#batch_normalization_106/moving_meanbatch_normalization_106/betadense_52/kerneldense_52/bias'batch_normalization_107/moving_variancebatch_normalization_107/gamma#batch_normalization_107/moving_meanbatch_normalization_107/betadense_53/kerneldense_53/bias*2
Tin+
)2'*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*H
_read_only_resource_inputs*
(&	
 !"#$%&*0
config_proto 

CPU

GPU2*0J 8? *-
f(R&
$__inference_signature_wrapper_584685
O
saver_filenamePlaceholder*
_output_shapes
: *
dtype0*
shape: 
?'
StatefulPartitionedCall_1StatefulPartitionedCallsaver_filename$conv2d_68/kernel/Read/ReadVariableOp"conv2d_68/bias/Read/ReadVariableOp1batch_normalization_102/gamma/Read/ReadVariableOp0batch_normalization_102/beta/Read/ReadVariableOp7batch_normalization_102/moving_mean/Read/ReadVariableOp;batch_normalization_102/moving_variance/Read/ReadVariableOp$conv2d_69/kernel/Read/ReadVariableOp"conv2d_69/bias/Read/ReadVariableOp1batch_normalization_103/gamma/Read/ReadVariableOp0batch_normalization_103/beta/Read/ReadVariableOp7batch_normalization_103/moving_mean/Read/ReadVariableOp;batch_normalization_103/moving_variance/Read/ReadVariableOp$conv2d_70/kernel/Read/ReadVariableOp"conv2d_70/bias/Read/ReadVariableOp1batch_normalization_104/gamma/Read/ReadVariableOp0batch_normalization_104/beta/Read/ReadVariableOp7batch_normalization_104/moving_mean/Read/ReadVariableOp;batch_normalization_104/moving_variance/Read/ReadVariableOp$conv2d_71/kernel/Read/ReadVariableOp"conv2d_71/bias/Read/ReadVariableOp1batch_normalization_105/gamma/Read/ReadVariableOp0batch_normalization_105/beta/Read/ReadVariableOp7batch_normalization_105/moving_mean/Read/ReadVariableOp;batch_normalization_105/moving_variance/Read/ReadVariableOp#dense_51/kernel/Read/ReadVariableOp!dense_51/bias/Read/ReadVariableOp1batch_normalization_106/gamma/Read/ReadVariableOp0batch_normalization_106/beta/Read/ReadVariableOp7batch_normalization_106/moving_mean/Read/ReadVariableOp;batch_normalization_106/moving_variance/Read/ReadVariableOp#dense_52/kernel/Read/ReadVariableOp!dense_52/bias/Read/ReadVariableOp1batch_normalization_107/gamma/Read/ReadVariableOp0batch_normalization_107/beta/Read/ReadVariableOp7batch_normalization_107/moving_mean/Read/ReadVariableOp;batch_normalization_107/moving_variance/Read/ReadVariableOp#dense_53/kernel/Read/ReadVariableOp!dense_53/bias/Read/ReadVariableOpAdam/iter/Read/ReadVariableOpAdam/beta_1/Read/ReadVariableOpAdam/beta_2/Read/ReadVariableOpAdam/decay/Read/ReadVariableOp&Adam/learning_rate/Read/ReadVariableOptotal/Read/ReadVariableOpcount/Read/ReadVariableOptotal_1/Read/ReadVariableOpcount_1/Read/ReadVariableOp+Adam/conv2d_68/kernel/m/Read/ReadVariableOp)Adam/conv2d_68/bias/m/Read/ReadVariableOp8Adam/batch_normalization_102/gamma/m/Read/ReadVariableOp7Adam/batch_normalization_102/beta/m/Read/ReadVariableOp+Adam/conv2d_69/kernel/m/Read/ReadVariableOp)Adam/conv2d_69/bias/m/Read/ReadVariableOp8Adam/batch_normalization_103/gamma/m/Read/ReadVariableOp7Adam/batch_normalization_103/beta/m/Read/ReadVariableOp+Adam/conv2d_70/kernel/m/Read/ReadVariableOp)Adam/conv2d_70/bias/m/Read/ReadVariableOp8Adam/batch_normalization_104/gamma/m/Read/ReadVariableOp7Adam/batch_normalization_104/beta/m/Read/ReadVariableOp+Adam/conv2d_71/kernel/m/Read/ReadVariableOp)Adam/conv2d_71/bias/m/Read/ReadVariableOp8Adam/batch_normalization_105/gamma/m/Read/ReadVariableOp7Adam/batch_normalization_105/beta/m/Read/ReadVariableOp*Adam/dense_51/kernel/m/Read/ReadVariableOp(Adam/dense_51/bias/m/Read/ReadVariableOp8Adam/batch_normalization_106/gamma/m/Read/ReadVariableOp7Adam/batch_normalization_106/beta/m/Read/ReadVariableOp*Adam/dense_52/kernel/m/Read/ReadVariableOp(Adam/dense_52/bias/m/Read/ReadVariableOp8Adam/batch_normalization_107/gamma/m/Read/ReadVariableOp7Adam/batch_normalization_107/beta/m/Read/ReadVariableOp*Adam/dense_53/kernel/m/Read/ReadVariableOp(Adam/dense_53/bias/m/Read/ReadVariableOp+Adam/conv2d_68/kernel/v/Read/ReadVariableOp)Adam/conv2d_68/bias/v/Read/ReadVariableOp8Adam/batch_normalization_102/gamma/v/Read/ReadVariableOp7Adam/batch_normalization_102/beta/v/Read/ReadVariableOp+Adam/conv2d_69/kernel/v/Read/ReadVariableOp)Adam/conv2d_69/bias/v/Read/ReadVariableOp8Adam/batch_normalization_103/gamma/v/Read/ReadVariableOp7Adam/batch_normalization_103/beta/v/Read/ReadVariableOp+Adam/conv2d_70/kernel/v/Read/ReadVariableOp)Adam/conv2d_70/bias/v/Read/ReadVariableOp8Adam/batch_normalization_104/gamma/v/Read/ReadVariableOp7Adam/batch_normalization_104/beta/v/Read/ReadVariableOp+Adam/conv2d_71/kernel/v/Read/ReadVariableOp)Adam/conv2d_71/bias/v/Read/ReadVariableOp8Adam/batch_normalization_105/gamma/v/Read/ReadVariableOp7Adam/batch_normalization_105/beta/v/Read/ReadVariableOp*Adam/dense_51/kernel/v/Read/ReadVariableOp(Adam/dense_51/bias/v/Read/ReadVariableOp8Adam/batch_normalization_106/gamma/v/Read/ReadVariableOp7Adam/batch_normalization_106/beta/v/Read/ReadVariableOp*Adam/dense_52/kernel/v/Read/ReadVariableOp(Adam/dense_52/bias/v/Read/ReadVariableOp8Adam/batch_normalization_107/gamma/v/Read/ReadVariableOp7Adam/batch_normalization_107/beta/v/Read/ReadVariableOp*Adam/dense_53/kernel/v/Read/ReadVariableOp(Adam/dense_53/bias/v/Read/ReadVariableOpConst*p
Tini
g2e	*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *(
f#R!
__inference__traced_save_586510
?
StatefulPartitionedCall_2StatefulPartitionedCallsaver_filenameconv2d_68/kernelconv2d_68/biasbatch_normalization_102/gammabatch_normalization_102/beta#batch_normalization_102/moving_mean'batch_normalization_102/moving_varianceconv2d_69/kernelconv2d_69/biasbatch_normalization_103/gammabatch_normalization_103/beta#batch_normalization_103/moving_mean'batch_normalization_103/moving_varianceconv2d_70/kernelconv2d_70/biasbatch_normalization_104/gammabatch_normalization_104/beta#batch_normalization_104/moving_mean'batch_normalization_104/moving_varianceconv2d_71/kernelconv2d_71/biasbatch_normalization_105/gammabatch_normalization_105/beta#batch_normalization_105/moving_mean'batch_normalization_105/moving_variancedense_51/kerneldense_51/biasbatch_normalization_106/gammabatch_normalization_106/beta#batch_normalization_106/moving_mean'batch_normalization_106/moving_variancedense_52/kerneldense_52/biasbatch_normalization_107/gammabatch_normalization_107/beta#batch_normalization_107/moving_mean'batch_normalization_107/moving_variancedense_53/kerneldense_53/bias	Adam/iterAdam/beta_1Adam/beta_2
Adam/decayAdam/learning_ratetotalcounttotal_1count_1Adam/conv2d_68/kernel/mAdam/conv2d_68/bias/m$Adam/batch_normalization_102/gamma/m#Adam/batch_normalization_102/beta/mAdam/conv2d_69/kernel/mAdam/conv2d_69/bias/m$Adam/batch_normalization_103/gamma/m#Adam/batch_normalization_103/beta/mAdam/conv2d_70/kernel/mAdam/conv2d_70/bias/m$Adam/batch_normalization_104/gamma/m#Adam/batch_normalization_104/beta/mAdam/conv2d_71/kernel/mAdam/conv2d_71/bias/m$Adam/batch_normalization_105/gamma/m#Adam/batch_normalization_105/beta/mAdam/dense_51/kernel/mAdam/dense_51/bias/m$Adam/batch_normalization_106/gamma/m#Adam/batch_normalization_106/beta/mAdam/dense_52/kernel/mAdam/dense_52/bias/m$Adam/batch_normalization_107/gamma/m#Adam/batch_normalization_107/beta/mAdam/dense_53/kernel/mAdam/dense_53/bias/mAdam/conv2d_68/kernel/vAdam/conv2d_68/bias/v$Adam/batch_normalization_102/gamma/v#Adam/batch_normalization_102/beta/vAdam/conv2d_69/kernel/vAdam/conv2d_69/bias/v$Adam/batch_normalization_103/gamma/v#Adam/batch_normalization_103/beta/vAdam/conv2d_70/kernel/vAdam/conv2d_70/bias/v$Adam/batch_normalization_104/gamma/v#Adam/batch_normalization_104/beta/vAdam/conv2d_71/kernel/vAdam/conv2d_71/bias/v$Adam/batch_normalization_105/gamma/v#Adam/batch_normalization_105/beta/vAdam/dense_51/kernel/vAdam/dense_51/bias/v$Adam/batch_normalization_106/gamma/v#Adam/batch_normalization_106/beta/vAdam/dense_52/kernel/vAdam/dense_52/bias/v$Adam/batch_normalization_107/gamma/v#Adam/batch_normalization_107/beta/vAdam/dense_53/kernel/vAdam/dense_53/bias/v*o
Tinh
f2d*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *+
f&R$
"__inference__traced_restore_586817??
?
e
F__inference_dropout_52_layer_call_and_return_conditional_losses_585997

inputs
identity?c
dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *UU??2
dropout/Constt
dropout/MulMulinputsdropout/Const:output:0*
T0*(
_output_shapes
:??????????2
dropout/MulT
dropout/ShapeShapeinputs*
T0*
_output_shapes
:2
dropout/Shape?
$dropout/random_uniform/RandomUniformRandomUniformdropout/Shape:output:0*
T0*(
_output_shapes
:??????????*
dtype02&
$dropout/random_uniform/RandomUniformu
dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *???>2
dropout/GreaterEqual/y?
dropout/GreaterEqualGreaterEqual-dropout/random_uniform/RandomUniform:output:0dropout/GreaterEqual/y:output:0*
T0*(
_output_shapes
:??????????2
dropout/GreaterEqual?
dropout/CastCastdropout/GreaterEqual:z:0*

DstT0*

SrcT0
*(
_output_shapes
:??????????2
dropout/Cast{
dropout/Mul_1Muldropout/Mul:z:0dropout/Cast:y:0*
T0*(
_output_shapes
:??????????2
dropout/Mul_1f
IdentityIdentitydropout/Mul_1:z:0*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
d
+__inference_dropout_53_layer_call_fn_586156

inputs
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_53_layer_call_and_return_conditional_losses_5840522
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????22
StatefulPartitionedCallStatefulPartitionedCall:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
f
J__inference_activation_125_layer_call_and_return_conditional_losses_584101

inputs
identityW
SoftmaxSoftmaxinputs*
T0*'
_output_shapes
:?????????2	
Softmaxe
IdentityIdentitySoftmax:softmax:0*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*&
_input_shapes
:?????????:O K
'
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_103_layer_call_fn_585458

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????88<*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_5835422
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*/
_output_shapes
:?????????88<2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????88<::::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????88<
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_102_layer_call_fn_585288

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 *A
_output_shapes/
-:+???????????????????????????<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_5826922
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*A
_output_shapes/
-:+???????????????????????????<2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????<::::22
StatefulPartitionedCallStatefulPartitionedCall:i e
A
_output_shapes/
-:+???????????????????????????<
 
_user_specified_nameinputs
?
f
J__inference_activation_124_layer_call_and_return_conditional_losses_583997

inputs
identityO
ReluReluinputs*
T0*(
_output_shapes
:??????????2
Relug
IdentityIdentityRelu:activations:0*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_106_layer_call_and_return_conditional_losses_583199

inputs%
!batchnorm_readvariableop_resource)
%batchnorm_mul_readvariableop_resource'
#batchnorm_readvariableop_1_resource'
#batchnorm_readvariableop_2_resource
identity??batchnorm/ReadVariableOp?batchnorm/ReadVariableOp_1?batchnorm/ReadVariableOp_2?batchnorm/mul/ReadVariableOp?
batchnorm/ReadVariableOpReadVariableOp!batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype02
batchnorm/ReadVariableOpg
batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:2
batchnorm/add/y?
batchnorm/addAddV2 batchnorm/ReadVariableOp:value:0batchnorm/add/y:output:0*
T0*
_output_shapes	
:?2
batchnorm/addd
batchnorm/RsqrtRsqrtbatchnorm/add:z:0*
T0*
_output_shapes	
:?2
batchnorm/Rsqrt?
batchnorm/mul/ReadVariableOpReadVariableOp%batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype02
batchnorm/mul/ReadVariableOp?
batchnorm/mulMulbatchnorm/Rsqrt:y:0$batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?2
batchnorm/mulw
batchnorm/mul_1Mulinputsbatchnorm/mul:z:0*
T0*(
_output_shapes
:??????????2
batchnorm/mul_1?
batchnorm/ReadVariableOp_1ReadVariableOp#batchnorm_readvariableop_1_resource*
_output_shapes	
:?*
dtype02
batchnorm/ReadVariableOp_1?
batchnorm/mul_2Mul"batchnorm/ReadVariableOp_1:value:0batchnorm/mul:z:0*
T0*
_output_shapes	
:?2
batchnorm/mul_2?
batchnorm/ReadVariableOp_2ReadVariableOp#batchnorm_readvariableop_2_resource*
_output_shapes	
:?*
dtype02
batchnorm/ReadVariableOp_2?
batchnorm/subSub"batchnorm/ReadVariableOp_2:value:0batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?2
batchnorm/sub?
batchnorm/add_1AddV2batchnorm/mul_1:z:0batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????2
batchnorm/add_1?
IdentityIdentitybatchnorm/add_1:z:0^batchnorm/ReadVariableOp^batchnorm/ReadVariableOp_1^batchnorm/ReadVariableOp_2^batchnorm/mul/ReadVariableOp*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*7
_input_shapes&
$:??????????::::24
batchnorm/ReadVariableOpbatchnorm/ReadVariableOp28
batchnorm/ReadVariableOp_1batchnorm/ReadVariableOp_128
batchnorm/ReadVariableOp_2batchnorm/ReadVariableOp_22<
batchnorm/mul/ReadVariableOpbatchnorm/mul/ReadVariableOp:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
G
+__inference_dropout_51_layer_call_fn_585863

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_51_layer_call_and_return_conditional_losses_5838212
PartitionedCallt
IdentityIdentityPartitionedCall:output:0*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_585635

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??AssignNewValue?AssignNewValue_1?FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
exponential_avg_factor%
?#<2
FusedBatchNormV3?
AssignNewValueAssignVariableOp(fusedbatchnormv3_readvariableop_resourceFusedBatchNormV3:batch_mean:0 ^FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*;
_class1
/-loc:@FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02
AssignNewValue?
AssignNewValue_1AssignVariableOp*fusedbatchnormv3_readvariableop_1_resource!FusedBatchNormV3:batch_variance:0"^FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*=
_class3
1/loc:@FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02
AssignNewValue_1?
IdentityIdentityFusedBatchNormV3:y:0^AssignNewValue^AssignNewValue_1 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????::::2 
AssignNewValueAssignNewValue2$
AssignNewValue_1AssignNewValue_12B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
??
?
I__inference_sequential_17_layer_call_and_return_conditional_losses_585046

inputs,
(conv2d_68_conv2d_readvariableop_resource-
)conv2d_68_biasadd_readvariableop_resource3
/batch_normalization_102_readvariableop_resource5
1batch_normalization_102_readvariableop_1_resourceD
@batch_normalization_102_fusedbatchnormv3_readvariableop_resourceF
Bbatch_normalization_102_fusedbatchnormv3_readvariableop_1_resource,
(conv2d_69_conv2d_readvariableop_resource-
)conv2d_69_biasadd_readvariableop_resource3
/batch_normalization_103_readvariableop_resource5
1batch_normalization_103_readvariableop_1_resourceD
@batch_normalization_103_fusedbatchnormv3_readvariableop_resourceF
Bbatch_normalization_103_fusedbatchnormv3_readvariableop_1_resource,
(conv2d_70_conv2d_readvariableop_resource-
)conv2d_70_biasadd_readvariableop_resource3
/batch_normalization_104_readvariableop_resource5
1batch_normalization_104_readvariableop_1_resourceD
@batch_normalization_104_fusedbatchnormv3_readvariableop_resourceF
Bbatch_normalization_104_fusedbatchnormv3_readvariableop_1_resource,
(conv2d_71_conv2d_readvariableop_resource-
)conv2d_71_biasadd_readvariableop_resource3
/batch_normalization_105_readvariableop_resource5
1batch_normalization_105_readvariableop_1_resourceD
@batch_normalization_105_fusedbatchnormv3_readvariableop_resourceF
Bbatch_normalization_105_fusedbatchnormv3_readvariableop_1_resource+
'dense_51_matmul_readvariableop_resource,
(dense_51_biasadd_readvariableop_resource=
9batch_normalization_106_batchnorm_readvariableop_resourceA
=batch_normalization_106_batchnorm_mul_readvariableop_resource?
;batch_normalization_106_batchnorm_readvariableop_1_resource?
;batch_normalization_106_batchnorm_readvariableop_2_resource+
'dense_52_matmul_readvariableop_resource,
(dense_52_biasadd_readvariableop_resource=
9batch_normalization_107_batchnorm_readvariableop_resourceA
=batch_normalization_107_batchnorm_mul_readvariableop_resource?
;batch_normalization_107_batchnorm_readvariableop_1_resource?
;batch_normalization_107_batchnorm_readvariableop_2_resource+
'dense_53_matmul_readvariableop_resource,
(dense_53_biasadd_readvariableop_resource
identity??7batch_normalization_102/FusedBatchNormV3/ReadVariableOp?9batch_normalization_102/FusedBatchNormV3/ReadVariableOp_1?&batch_normalization_102/ReadVariableOp?(batch_normalization_102/ReadVariableOp_1?7batch_normalization_103/FusedBatchNormV3/ReadVariableOp?9batch_normalization_103/FusedBatchNormV3/ReadVariableOp_1?&batch_normalization_103/ReadVariableOp?(batch_normalization_103/ReadVariableOp_1?7batch_normalization_104/FusedBatchNormV3/ReadVariableOp?9batch_normalization_104/FusedBatchNormV3/ReadVariableOp_1?&batch_normalization_104/ReadVariableOp?(batch_normalization_104/ReadVariableOp_1?7batch_normalization_105/FusedBatchNormV3/ReadVariableOp?9batch_normalization_105/FusedBatchNormV3/ReadVariableOp_1?&batch_normalization_105/ReadVariableOp?(batch_normalization_105/ReadVariableOp_1?0batch_normalization_106/batchnorm/ReadVariableOp?2batch_normalization_106/batchnorm/ReadVariableOp_1?2batch_normalization_106/batchnorm/ReadVariableOp_2?4batch_normalization_106/batchnorm/mul/ReadVariableOp?0batch_normalization_107/batchnorm/ReadVariableOp?2batch_normalization_107/batchnorm/ReadVariableOp_1?2batch_normalization_107/batchnorm/ReadVariableOp_2?4batch_normalization_107/batchnorm/mul/ReadVariableOp? conv2d_68/BiasAdd/ReadVariableOp?conv2d_68/Conv2D/ReadVariableOp? conv2d_69/BiasAdd/ReadVariableOp?conv2d_69/Conv2D/ReadVariableOp? conv2d_70/BiasAdd/ReadVariableOp?conv2d_70/Conv2D/ReadVariableOp? conv2d_71/BiasAdd/ReadVariableOp?conv2d_71/Conv2D/ReadVariableOp?dense_51/BiasAdd/ReadVariableOp?dense_51/MatMul/ReadVariableOp?dense_52/BiasAdd/ReadVariableOp?dense_52/MatMul/ReadVariableOp?dense_53/BiasAdd/ReadVariableOp?dense_53/MatMul/ReadVariableOp?
conv2d_68/Conv2D/ReadVariableOpReadVariableOp(conv2d_68_conv2d_readvariableop_resource*&
_output_shapes
:<*
dtype02!
conv2d_68/Conv2D/ReadVariableOp?
conv2d_68/Conv2DConv2Dinputs'conv2d_68/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????<<<*
paddingVALID*
strides
2
conv2d_68/Conv2D?
 conv2d_68/BiasAdd/ReadVariableOpReadVariableOp)conv2d_68_biasadd_readvariableop_resource*
_output_shapes
:<*
dtype02"
 conv2d_68/BiasAdd/ReadVariableOp?
conv2d_68/BiasAddBiasAddconv2d_68/Conv2D:output:0(conv2d_68/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????<<<2
conv2d_68/BiasAdd?
activation_119/ReluReluconv2d_68/BiasAdd:output:0*
T0*/
_output_shapes
:?????????<<<2
activation_119/Relu?
&batch_normalization_102/ReadVariableOpReadVariableOp/batch_normalization_102_readvariableop_resource*
_output_shapes
:<*
dtype02(
&batch_normalization_102/ReadVariableOp?
(batch_normalization_102/ReadVariableOp_1ReadVariableOp1batch_normalization_102_readvariableop_1_resource*
_output_shapes
:<*
dtype02*
(batch_normalization_102/ReadVariableOp_1?
7batch_normalization_102/FusedBatchNormV3/ReadVariableOpReadVariableOp@batch_normalization_102_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype029
7batch_normalization_102/FusedBatchNormV3/ReadVariableOp?
9batch_normalization_102/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpBbatch_normalization_102_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02;
9batch_normalization_102/FusedBatchNormV3/ReadVariableOp_1?
(batch_normalization_102/FusedBatchNormV3FusedBatchNormV3!activation_119/Relu:activations:0.batch_normalization_102/ReadVariableOp:value:00batch_normalization_102/ReadVariableOp_1:value:0?batch_normalization_102/FusedBatchNormV3/ReadVariableOp:value:0Abatch_normalization_102/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????<<<:<:<:<:<:*
epsilon%o?:*
is_training( 2*
(batch_normalization_102/FusedBatchNormV3?
conv2d_69/Conv2D/ReadVariableOpReadVariableOp(conv2d_69_conv2d_readvariableop_resource*&
_output_shapes
:<<*
dtype02!
conv2d_69/Conv2D/ReadVariableOp?
conv2d_69/Conv2DConv2D,batch_normalization_102/FusedBatchNormV3:y:0'conv2d_69/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????88<*
paddingVALID*
strides
2
conv2d_69/Conv2D?
 conv2d_69/BiasAdd/ReadVariableOpReadVariableOp)conv2d_69_biasadd_readvariableop_resource*
_output_shapes
:<*
dtype02"
 conv2d_69/BiasAdd/ReadVariableOp?
conv2d_69/BiasAddBiasAddconv2d_69/Conv2D:output:0(conv2d_69/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????88<2
conv2d_69/BiasAdd?
activation_120/ReluReluconv2d_69/BiasAdd:output:0*
T0*/
_output_shapes
:?????????88<2
activation_120/Relu?
&batch_normalization_103/ReadVariableOpReadVariableOp/batch_normalization_103_readvariableop_resource*
_output_shapes
:<*
dtype02(
&batch_normalization_103/ReadVariableOp?
(batch_normalization_103/ReadVariableOp_1ReadVariableOp1batch_normalization_103_readvariableop_1_resource*
_output_shapes
:<*
dtype02*
(batch_normalization_103/ReadVariableOp_1?
7batch_normalization_103/FusedBatchNormV3/ReadVariableOpReadVariableOp@batch_normalization_103_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype029
7batch_normalization_103/FusedBatchNormV3/ReadVariableOp?
9batch_normalization_103/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpBbatch_normalization_103_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02;
9batch_normalization_103/FusedBatchNormV3/ReadVariableOp_1?
(batch_normalization_103/FusedBatchNormV3FusedBatchNormV3!activation_120/Relu:activations:0.batch_normalization_103/ReadVariableOp:value:00batch_normalization_103/ReadVariableOp_1:value:0?batch_normalization_103/FusedBatchNormV3/ReadVariableOp:value:0Abatch_normalization_103/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????88<:<:<:<:<:*
epsilon%o?:*
is_training( 2*
(batch_normalization_103/FusedBatchNormV3?
max_pooling2d_34/MaxPoolMaxPool,batch_normalization_103/FusedBatchNormV3:y:0*/
_output_shapes
:?????????<*
ksize
*
paddingVALID*
strides
2
max_pooling2d_34/MaxPool?
conv2d_70/Conv2D/ReadVariableOpReadVariableOp(conv2d_70_conv2d_readvariableop_resource*&
_output_shapes
:<*
dtype02!
conv2d_70/Conv2D/ReadVariableOp?
conv2d_70/Conv2DConv2D!max_pooling2d_34/MaxPool:output:0'conv2d_70/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2
conv2d_70/Conv2D?
 conv2d_70/BiasAdd/ReadVariableOpReadVariableOp)conv2d_70_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02"
 conv2d_70/BiasAdd/ReadVariableOp?
conv2d_70/BiasAddBiasAddconv2d_70/Conv2D:output:0(conv2d_70/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????2
conv2d_70/BiasAdd?
activation_121/ReluReluconv2d_70/BiasAdd:output:0*
T0*/
_output_shapes
:?????????2
activation_121/Relu?
&batch_normalization_104/ReadVariableOpReadVariableOp/batch_normalization_104_readvariableop_resource*
_output_shapes
:*
dtype02(
&batch_normalization_104/ReadVariableOp?
(batch_normalization_104/ReadVariableOp_1ReadVariableOp1batch_normalization_104_readvariableop_1_resource*
_output_shapes
:*
dtype02*
(batch_normalization_104/ReadVariableOp_1?
7batch_normalization_104/FusedBatchNormV3/ReadVariableOpReadVariableOp@batch_normalization_104_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype029
7batch_normalization_104/FusedBatchNormV3/ReadVariableOp?
9batch_normalization_104/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpBbatch_normalization_104_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02;
9batch_normalization_104/FusedBatchNormV3/ReadVariableOp_1?
(batch_normalization_104/FusedBatchNormV3FusedBatchNormV3!activation_121/Relu:activations:0.batch_normalization_104/ReadVariableOp:value:00batch_normalization_104/ReadVariableOp_1:value:0?batch_normalization_104/FusedBatchNormV3/ReadVariableOp:value:0Abatch_normalization_104/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
is_training( 2*
(batch_normalization_104/FusedBatchNormV3?
conv2d_71/Conv2D/ReadVariableOpReadVariableOp(conv2d_71_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02!
conv2d_71/Conv2D/ReadVariableOp?
conv2d_71/Conv2DConv2D,batch_normalization_104/FusedBatchNormV3:y:0'conv2d_71/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2
conv2d_71/Conv2D?
 conv2d_71/BiasAdd/ReadVariableOpReadVariableOp)conv2d_71_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02"
 conv2d_71/BiasAdd/ReadVariableOp?
conv2d_71/BiasAddBiasAddconv2d_71/Conv2D:output:0(conv2d_71/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????2
conv2d_71/BiasAdd?
activation_122/ReluReluconv2d_71/BiasAdd:output:0*
T0*/
_output_shapes
:?????????2
activation_122/Relu?
&batch_normalization_105/ReadVariableOpReadVariableOp/batch_normalization_105_readvariableop_resource*
_output_shapes
:*
dtype02(
&batch_normalization_105/ReadVariableOp?
(batch_normalization_105/ReadVariableOp_1ReadVariableOp1batch_normalization_105_readvariableop_1_resource*
_output_shapes
:*
dtype02*
(batch_normalization_105/ReadVariableOp_1?
7batch_normalization_105/FusedBatchNormV3/ReadVariableOpReadVariableOp@batch_normalization_105_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype029
7batch_normalization_105/FusedBatchNormV3/ReadVariableOp?
9batch_normalization_105/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpBbatch_normalization_105_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02;
9batch_normalization_105/FusedBatchNormV3/ReadVariableOp_1?
(batch_normalization_105/FusedBatchNormV3FusedBatchNormV3!activation_122/Relu:activations:0.batch_normalization_105/ReadVariableOp:value:00batch_normalization_105/ReadVariableOp_1:value:0?batch_normalization_105/FusedBatchNormV3/ReadVariableOp:value:0Abatch_normalization_105/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
is_training( 2*
(batch_normalization_105/FusedBatchNormV3?
max_pooling2d_35/MaxPoolMaxPool,batch_normalization_105/FusedBatchNormV3:y:0*/
_output_shapes
:?????????*
ksize
*
paddingVALID*
strides
2
max_pooling2d_35/MaxPool?
dropout_51/IdentityIdentity!max_pooling2d_35/MaxPool:output:0*
T0*/
_output_shapes
:?????????2
dropout_51/Identityu
flatten_34/ConstConst*
_output_shapes
:*
dtype0*
valueB"?????  2
flatten_34/Const?
flatten_34/ReshapeReshapedropout_51/Identity:output:0flatten_34/Const:output:0*
T0*(
_output_shapes
:??????????!2
flatten_34/Reshape?
dense_51/MatMul/ReadVariableOpReadVariableOp'dense_51_matmul_readvariableop_resource* 
_output_shapes
:
?!?*
dtype02 
dense_51/MatMul/ReadVariableOp?
dense_51/MatMulMatMulflatten_34/Reshape:output:0&dense_51/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
dense_51/MatMul?
dense_51/BiasAdd/ReadVariableOpReadVariableOp(dense_51_biasadd_readvariableop_resource*
_output_shapes	
:?*
dtype02!
dense_51/BiasAdd/ReadVariableOp?
dense_51/BiasAddBiasAdddense_51/MatMul:product:0'dense_51/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
dense_51/BiasAdd?
activation_123/ReluReludense_51/BiasAdd:output:0*
T0*(
_output_shapes
:??????????2
activation_123/Relu?
0batch_normalization_106/batchnorm/ReadVariableOpReadVariableOp9batch_normalization_106_batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype022
0batch_normalization_106/batchnorm/ReadVariableOp?
'batch_normalization_106/batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:2)
'batch_normalization_106/batchnorm/add/y?
%batch_normalization_106/batchnorm/addAddV28batch_normalization_106/batchnorm/ReadVariableOp:value:00batch_normalization_106/batchnorm/add/y:output:0*
T0*
_output_shapes	
:?2'
%batch_normalization_106/batchnorm/add?
'batch_normalization_106/batchnorm/RsqrtRsqrt)batch_normalization_106/batchnorm/add:z:0*
T0*
_output_shapes	
:?2)
'batch_normalization_106/batchnorm/Rsqrt?
4batch_normalization_106/batchnorm/mul/ReadVariableOpReadVariableOp=batch_normalization_106_batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype026
4batch_normalization_106/batchnorm/mul/ReadVariableOp?
%batch_normalization_106/batchnorm/mulMul+batch_normalization_106/batchnorm/Rsqrt:y:0<batch_normalization_106/batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?2'
%batch_normalization_106/batchnorm/mul?
'batch_normalization_106/batchnorm/mul_1Mul!activation_123/Relu:activations:0)batch_normalization_106/batchnorm/mul:z:0*
T0*(
_output_shapes
:??????????2)
'batch_normalization_106/batchnorm/mul_1?
2batch_normalization_106/batchnorm/ReadVariableOp_1ReadVariableOp;batch_normalization_106_batchnorm_readvariableop_1_resource*
_output_shapes	
:?*
dtype024
2batch_normalization_106/batchnorm/ReadVariableOp_1?
'batch_normalization_106/batchnorm/mul_2Mul:batch_normalization_106/batchnorm/ReadVariableOp_1:value:0)batch_normalization_106/batchnorm/mul:z:0*
T0*
_output_shapes	
:?2)
'batch_normalization_106/batchnorm/mul_2?
2batch_normalization_106/batchnorm/ReadVariableOp_2ReadVariableOp;batch_normalization_106_batchnorm_readvariableop_2_resource*
_output_shapes	
:?*
dtype024
2batch_normalization_106/batchnorm/ReadVariableOp_2?
%batch_normalization_106/batchnorm/subSub:batch_normalization_106/batchnorm/ReadVariableOp_2:value:0+batch_normalization_106/batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?2'
%batch_normalization_106/batchnorm/sub?
'batch_normalization_106/batchnorm/add_1AddV2+batch_normalization_106/batchnorm/mul_1:z:0)batch_normalization_106/batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????2)
'batch_normalization_106/batchnorm/add_1?
dropout_52/IdentityIdentity+batch_normalization_106/batchnorm/add_1:z:0*
T0*(
_output_shapes
:??????????2
dropout_52/Identityu
flatten_35/ConstConst*
_output_shapes
:*
dtype0*
valueB"????,  2
flatten_35/Const?
flatten_35/ReshapeReshapedropout_52/Identity:output:0flatten_35/Const:output:0*
T0*(
_output_shapes
:??????????2
flatten_35/Reshape?
dense_52/MatMul/ReadVariableOpReadVariableOp'dense_52_matmul_readvariableop_resource* 
_output_shapes
:
??*
dtype02 
dense_52/MatMul/ReadVariableOp?
dense_52/MatMulMatMulflatten_35/Reshape:output:0&dense_52/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
dense_52/MatMul?
dense_52/BiasAdd/ReadVariableOpReadVariableOp(dense_52_biasadd_readvariableop_resource*
_output_shapes	
:?*
dtype02!
dense_52/BiasAdd/ReadVariableOp?
dense_52/BiasAddBiasAdddense_52/MatMul:product:0'dense_52/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
dense_52/BiasAdd?
activation_124/ReluReludense_52/BiasAdd:output:0*
T0*(
_output_shapes
:??????????2
activation_124/Relu?
0batch_normalization_107/batchnorm/ReadVariableOpReadVariableOp9batch_normalization_107_batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype022
0batch_normalization_107/batchnorm/ReadVariableOp?
'batch_normalization_107/batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:2)
'batch_normalization_107/batchnorm/add/y?
%batch_normalization_107/batchnorm/addAddV28batch_normalization_107/batchnorm/ReadVariableOp:value:00batch_normalization_107/batchnorm/add/y:output:0*
T0*
_output_shapes	
:?2'
%batch_normalization_107/batchnorm/add?
'batch_normalization_107/batchnorm/RsqrtRsqrt)batch_normalization_107/batchnorm/add:z:0*
T0*
_output_shapes	
:?2)
'batch_normalization_107/batchnorm/Rsqrt?
4batch_normalization_107/batchnorm/mul/ReadVariableOpReadVariableOp=batch_normalization_107_batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype026
4batch_normalization_107/batchnorm/mul/ReadVariableOp?
%batch_normalization_107/batchnorm/mulMul+batch_normalization_107/batchnorm/Rsqrt:y:0<batch_normalization_107/batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?2'
%batch_normalization_107/batchnorm/mul?
'batch_normalization_107/batchnorm/mul_1Mul!activation_124/Relu:activations:0)batch_normalization_107/batchnorm/mul:z:0*
T0*(
_output_shapes
:??????????2)
'batch_normalization_107/batchnorm/mul_1?
2batch_normalization_107/batchnorm/ReadVariableOp_1ReadVariableOp;batch_normalization_107_batchnorm_readvariableop_1_resource*
_output_shapes	
:?*
dtype024
2batch_normalization_107/batchnorm/ReadVariableOp_1?
'batch_normalization_107/batchnorm/mul_2Mul:batch_normalization_107/batchnorm/ReadVariableOp_1:value:0)batch_normalization_107/batchnorm/mul:z:0*
T0*
_output_shapes	
:?2)
'batch_normalization_107/batchnorm/mul_2?
2batch_normalization_107/batchnorm/ReadVariableOp_2ReadVariableOp;batch_normalization_107_batchnorm_readvariableop_2_resource*
_output_shapes	
:?*
dtype024
2batch_normalization_107/batchnorm/ReadVariableOp_2?
%batch_normalization_107/batchnorm/subSub:batch_normalization_107/batchnorm/ReadVariableOp_2:value:0+batch_normalization_107/batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?2'
%batch_normalization_107/batchnorm/sub?
'batch_normalization_107/batchnorm/add_1AddV2+batch_normalization_107/batchnorm/mul_1:z:0)batch_normalization_107/batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????2)
'batch_normalization_107/batchnorm/add_1?
dropout_53/IdentityIdentity+batch_normalization_107/batchnorm/add_1:z:0*
T0*(
_output_shapes
:??????????2
dropout_53/Identity?
dense_53/MatMul/ReadVariableOpReadVariableOp'dense_53_matmul_readvariableop_resource*
_output_shapes
:	?*
dtype02 
dense_53/MatMul/ReadVariableOp?
dense_53/MatMulMatMuldropout_53/Identity:output:0&dense_53/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
dense_53/MatMul?
dense_53/BiasAdd/ReadVariableOpReadVariableOp(dense_53_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02!
dense_53/BiasAdd/ReadVariableOp?
dense_53/BiasAddBiasAdddense_53/MatMul:product:0'dense_53/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
dense_53/BiasAdd?
activation_125/SoftmaxSoftmaxdense_53/BiasAdd:output:0*
T0*'
_output_shapes
:?????????2
activation_125/Softmax?
IdentityIdentity activation_125/Softmax:softmax:08^batch_normalization_102/FusedBatchNormV3/ReadVariableOp:^batch_normalization_102/FusedBatchNormV3/ReadVariableOp_1'^batch_normalization_102/ReadVariableOp)^batch_normalization_102/ReadVariableOp_18^batch_normalization_103/FusedBatchNormV3/ReadVariableOp:^batch_normalization_103/FusedBatchNormV3/ReadVariableOp_1'^batch_normalization_103/ReadVariableOp)^batch_normalization_103/ReadVariableOp_18^batch_normalization_104/FusedBatchNormV3/ReadVariableOp:^batch_normalization_104/FusedBatchNormV3/ReadVariableOp_1'^batch_normalization_104/ReadVariableOp)^batch_normalization_104/ReadVariableOp_18^batch_normalization_105/FusedBatchNormV3/ReadVariableOp:^batch_normalization_105/FusedBatchNormV3/ReadVariableOp_1'^batch_normalization_105/ReadVariableOp)^batch_normalization_105/ReadVariableOp_11^batch_normalization_106/batchnorm/ReadVariableOp3^batch_normalization_106/batchnorm/ReadVariableOp_13^batch_normalization_106/batchnorm/ReadVariableOp_25^batch_normalization_106/batchnorm/mul/ReadVariableOp1^batch_normalization_107/batchnorm/ReadVariableOp3^batch_normalization_107/batchnorm/ReadVariableOp_13^batch_normalization_107/batchnorm/ReadVariableOp_25^batch_normalization_107/batchnorm/mul/ReadVariableOp!^conv2d_68/BiasAdd/ReadVariableOp ^conv2d_68/Conv2D/ReadVariableOp!^conv2d_69/BiasAdd/ReadVariableOp ^conv2d_69/Conv2D/ReadVariableOp!^conv2d_70/BiasAdd/ReadVariableOp ^conv2d_70/Conv2D/ReadVariableOp!^conv2d_71/BiasAdd/ReadVariableOp ^conv2d_71/Conv2D/ReadVariableOp ^dense_51/BiasAdd/ReadVariableOp^dense_51/MatMul/ReadVariableOp ^dense_52/BiasAdd/ReadVariableOp^dense_52/MatMul/ReadVariableOp ^dense_53/BiasAdd/ReadVariableOp^dense_53/MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::2r
7batch_normalization_102/FusedBatchNormV3/ReadVariableOp7batch_normalization_102/FusedBatchNormV3/ReadVariableOp2v
9batch_normalization_102/FusedBatchNormV3/ReadVariableOp_19batch_normalization_102/FusedBatchNormV3/ReadVariableOp_12P
&batch_normalization_102/ReadVariableOp&batch_normalization_102/ReadVariableOp2T
(batch_normalization_102/ReadVariableOp_1(batch_normalization_102/ReadVariableOp_12r
7batch_normalization_103/FusedBatchNormV3/ReadVariableOp7batch_normalization_103/FusedBatchNormV3/ReadVariableOp2v
9batch_normalization_103/FusedBatchNormV3/ReadVariableOp_19batch_normalization_103/FusedBatchNormV3/ReadVariableOp_12P
&batch_normalization_103/ReadVariableOp&batch_normalization_103/ReadVariableOp2T
(batch_normalization_103/ReadVariableOp_1(batch_normalization_103/ReadVariableOp_12r
7batch_normalization_104/FusedBatchNormV3/ReadVariableOp7batch_normalization_104/FusedBatchNormV3/ReadVariableOp2v
9batch_normalization_104/FusedBatchNormV3/ReadVariableOp_19batch_normalization_104/FusedBatchNormV3/ReadVariableOp_12P
&batch_normalization_104/ReadVariableOp&batch_normalization_104/ReadVariableOp2T
(batch_normalization_104/ReadVariableOp_1(batch_normalization_104/ReadVariableOp_12r
7batch_normalization_105/FusedBatchNormV3/ReadVariableOp7batch_normalization_105/FusedBatchNormV3/ReadVariableOp2v
9batch_normalization_105/FusedBatchNormV3/ReadVariableOp_19batch_normalization_105/FusedBatchNormV3/ReadVariableOp_12P
&batch_normalization_105/ReadVariableOp&batch_normalization_105/ReadVariableOp2T
(batch_normalization_105/ReadVariableOp_1(batch_normalization_105/ReadVariableOp_12d
0batch_normalization_106/batchnorm/ReadVariableOp0batch_normalization_106/batchnorm/ReadVariableOp2h
2batch_normalization_106/batchnorm/ReadVariableOp_12batch_normalization_106/batchnorm/ReadVariableOp_12h
2batch_normalization_106/batchnorm/ReadVariableOp_22batch_normalization_106/batchnorm/ReadVariableOp_22l
4batch_normalization_106/batchnorm/mul/ReadVariableOp4batch_normalization_106/batchnorm/mul/ReadVariableOp2d
0batch_normalization_107/batchnorm/ReadVariableOp0batch_normalization_107/batchnorm/ReadVariableOp2h
2batch_normalization_107/batchnorm/ReadVariableOp_12batch_normalization_107/batchnorm/ReadVariableOp_12h
2batch_normalization_107/batchnorm/ReadVariableOp_22batch_normalization_107/batchnorm/ReadVariableOp_22l
4batch_normalization_107/batchnorm/mul/ReadVariableOp4batch_normalization_107/batchnorm/mul/ReadVariableOp2D
 conv2d_68/BiasAdd/ReadVariableOp conv2d_68/BiasAdd/ReadVariableOp2B
conv2d_68/Conv2D/ReadVariableOpconv2d_68/Conv2D/ReadVariableOp2D
 conv2d_69/BiasAdd/ReadVariableOp conv2d_69/BiasAdd/ReadVariableOp2B
conv2d_69/Conv2D/ReadVariableOpconv2d_69/Conv2D/ReadVariableOp2D
 conv2d_70/BiasAdd/ReadVariableOp conv2d_70/BiasAdd/ReadVariableOp2B
conv2d_70/Conv2D/ReadVariableOpconv2d_70/Conv2D/ReadVariableOp2D
 conv2d_71/BiasAdd/ReadVariableOp conv2d_71/BiasAdd/ReadVariableOp2B
conv2d_71/Conv2D/ReadVariableOpconv2d_71/Conv2D/ReadVariableOp2B
dense_51/BiasAdd/ReadVariableOpdense_51/BiasAdd/ReadVariableOp2@
dense_51/MatMul/ReadVariableOpdense_51/MatMul/ReadVariableOp2B
dense_52/BiasAdd/ReadVariableOpdense_52/BiasAdd/ReadVariableOp2@
dense_52/MatMul/ReadVariableOpdense_52/MatMul/ReadVariableOp2B
dense_53/BiasAdd/ReadVariableOpdense_53/BiasAdd/ReadVariableOp2@
dense_53/MatMul/ReadVariableOpdense_53/MatMul/ReadVariableOp:W S
/
_output_shapes
:?????????@@
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_585653

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
is_training( 2
FusedBatchNormV3?
IdentityIdentityFusedBatchNormV3:y:0 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????::::2B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_585275

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:<*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:<*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*]
_output_shapesK
I:+???????????????????????????<:<:<:<:<:*
epsilon%o?:*
is_training( 2
FusedBatchNormV3?
IdentityIdentityFusedBatchNormV3:y:0 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*A
_output_shapes/
-:+???????????????????????????<2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????<::::2B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:i e
A
_output_shapes/
-:+???????????????????????????<
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_585414

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??AssignNewValue?AssignNewValue_1?FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:<*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:<*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????88<:<:<:<:<:*
epsilon%o?:*
exponential_avg_factor%
?#<2
FusedBatchNormV3?
AssignNewValueAssignVariableOp(fusedbatchnormv3_readvariableop_resourceFusedBatchNormV3:batch_mean:0 ^FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*;
_class1
/-loc:@FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02
AssignNewValue?
AssignNewValue_1AssignVariableOp*fusedbatchnormv3_readvariableop_1_resource!FusedBatchNormV3:batch_variance:0"^FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*=
_class3
1/loc:@FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02
AssignNewValue_1?
IdentityIdentityFusedBatchNormV3:y:0^AssignNewValue^AssignNewValue_1 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*/
_output_shapes
:?????????88<2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????88<::::2 
AssignNewValueAssignNewValue2$
AssignNewValue_1AssignNewValue_12B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:W S
/
_output_shapes
:?????????88<
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_107_layer_call_and_return_conditional_losses_583339

inputs%
!batchnorm_readvariableop_resource)
%batchnorm_mul_readvariableop_resource'
#batchnorm_readvariableop_1_resource'
#batchnorm_readvariableop_2_resource
identity??batchnorm/ReadVariableOp?batchnorm/ReadVariableOp_1?batchnorm/ReadVariableOp_2?batchnorm/mul/ReadVariableOp?
batchnorm/ReadVariableOpReadVariableOp!batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype02
batchnorm/ReadVariableOpg
batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:2
batchnorm/add/y?
batchnorm/addAddV2 batchnorm/ReadVariableOp:value:0batchnorm/add/y:output:0*
T0*
_output_shapes	
:?2
batchnorm/addd
batchnorm/RsqrtRsqrtbatchnorm/add:z:0*
T0*
_output_shapes	
:?2
batchnorm/Rsqrt?
batchnorm/mul/ReadVariableOpReadVariableOp%batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype02
batchnorm/mul/ReadVariableOp?
batchnorm/mulMulbatchnorm/Rsqrt:y:0$batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?2
batchnorm/mulw
batchnorm/mul_1Mulinputsbatchnorm/mul:z:0*
T0*(
_output_shapes
:??????????2
batchnorm/mul_1?
batchnorm/ReadVariableOp_1ReadVariableOp#batchnorm_readvariableop_1_resource*
_output_shapes	
:?*
dtype02
batchnorm/ReadVariableOp_1?
batchnorm/mul_2Mul"batchnorm/ReadVariableOp_1:value:0batchnorm/mul:z:0*
T0*
_output_shapes	
:?2
batchnorm/mul_2?
batchnorm/ReadVariableOp_2ReadVariableOp#batchnorm_readvariableop_2_resource*
_output_shapes	
:?*
dtype02
batchnorm/ReadVariableOp_2?
batchnorm/subSub"batchnorm/ReadVariableOp_2:value:0batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?2
batchnorm/sub?
batchnorm/add_1AddV2batchnorm/mul_1:z:0batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????2
batchnorm/add_1?
IdentityIdentitybatchnorm/add_1:z:0^batchnorm/ReadVariableOp^batchnorm/ReadVariableOp_1^batchnorm/ReadVariableOp_2^batchnorm/mul/ReadVariableOp*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*7
_input_shapes&
$:??????????::::24
batchnorm/ReadVariableOpbatchnorm/ReadVariableOp28
batchnorm/ReadVariableOp_1batchnorm/ReadVariableOp_128
batchnorm/ReadVariableOp_2batchnorm/ReadVariableOp_22<
batchnorm/mul/ReadVariableOpbatchnorm/mul/ReadVariableOp:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?	
?
E__inference_conv2d_68_layer_call_and_return_conditional_losses_585218

inputs"
conv2d_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?Conv2D/ReadVariableOp?
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
:<*
dtype02
Conv2D/ReadVariableOp?
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????<<<*
paddingVALID*
strides
2
Conv2D?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:<*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????<<<2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*
T0*/
_output_shapes
:?????????<<<2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????@@::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:?????????@@
 
_user_specified_nameinputs
?0
?
S__inference_batch_normalization_107_layer_call_and_return_conditional_losses_586088

inputs
assignmovingavg_586063
assignmovingavg_1_586069)
%batchnorm_mul_readvariableop_resource%
!batchnorm_readvariableop_resource
identity??#AssignMovingAvg/AssignSubVariableOp?AssignMovingAvg/ReadVariableOp?%AssignMovingAvg_1/AssignSubVariableOp? AssignMovingAvg_1/ReadVariableOp?batchnorm/ReadVariableOp?batchnorm/mul/ReadVariableOp?
moments/mean/reduction_indicesConst*
_output_shapes
:*
dtype0*
valueB: 2 
moments/mean/reduction_indices?
moments/meanMeaninputs'moments/mean/reduction_indices:output:0*
T0*
_output_shapes
:	?*
	keep_dims(2
moments/mean}
moments/StopGradientStopGradientmoments/mean:output:0*
T0*
_output_shapes
:	?2
moments/StopGradient?
moments/SquaredDifferenceSquaredDifferenceinputsmoments/StopGradient:output:0*
T0*(
_output_shapes
:??????????2
moments/SquaredDifference?
"moments/variance/reduction_indicesConst*
_output_shapes
:*
dtype0*
valueB: 2$
"moments/variance/reduction_indices?
moments/varianceMeanmoments/SquaredDifference:z:0+moments/variance/reduction_indices:output:0*
T0*
_output_shapes
:	?*
	keep_dims(2
moments/variance?
moments/SqueezeSqueezemoments/mean:output:0*
T0*
_output_shapes	
:?*
squeeze_dims
 2
moments/Squeeze?
moments/Squeeze_1Squeezemoments/variance:output:0*
T0*
_output_shapes	
:?*
squeeze_dims
 2
moments/Squeeze_1?
AssignMovingAvg/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*)
_class
loc:@AssignMovingAvg/586063*
_output_shapes
: *
dtype0*
valueB
 *
?#<2
AssignMovingAvg/decay?
AssignMovingAvg/ReadVariableOpReadVariableOpassignmovingavg_586063*
_output_shapes	
:?*
dtype02 
AssignMovingAvg/ReadVariableOp?
AssignMovingAvg/subSub&AssignMovingAvg/ReadVariableOp:value:0moments/Squeeze:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*)
_class
loc:@AssignMovingAvg/586063*
_output_shapes	
:?2
AssignMovingAvg/sub?
AssignMovingAvg/mulMulAssignMovingAvg/sub:z:0AssignMovingAvg/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*)
_class
loc:@AssignMovingAvg/586063*
_output_shapes	
:?2
AssignMovingAvg/mul?
#AssignMovingAvg/AssignSubVariableOpAssignSubVariableOpassignmovingavg_586063AssignMovingAvg/mul:z:0^AssignMovingAvg/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*)
_class
loc:@AssignMovingAvg/586063*
_output_shapes
 *
dtype02%
#AssignMovingAvg/AssignSubVariableOp?
AssignMovingAvg_1/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*+
_class!
loc:@AssignMovingAvg_1/586069*
_output_shapes
: *
dtype0*
valueB
 *
?#<2
AssignMovingAvg_1/decay?
 AssignMovingAvg_1/ReadVariableOpReadVariableOpassignmovingavg_1_586069*
_output_shapes	
:?*
dtype02"
 AssignMovingAvg_1/ReadVariableOp?
AssignMovingAvg_1/subSub(AssignMovingAvg_1/ReadVariableOp:value:0moments/Squeeze_1:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*+
_class!
loc:@AssignMovingAvg_1/586069*
_output_shapes	
:?2
AssignMovingAvg_1/sub?
AssignMovingAvg_1/mulMulAssignMovingAvg_1/sub:z:0 AssignMovingAvg_1/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*+
_class!
loc:@AssignMovingAvg_1/586069*
_output_shapes	
:?2
AssignMovingAvg_1/mul?
%AssignMovingAvg_1/AssignSubVariableOpAssignSubVariableOpassignmovingavg_1_586069AssignMovingAvg_1/mul:z:0!^AssignMovingAvg_1/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*+
_class!
loc:@AssignMovingAvg_1/586069*
_output_shapes
 *
dtype02'
%AssignMovingAvg_1/AssignSubVariableOpg
batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:2
batchnorm/add/y?
batchnorm/addAddV2moments/Squeeze_1:output:0batchnorm/add/y:output:0*
T0*
_output_shapes	
:?2
batchnorm/addd
batchnorm/RsqrtRsqrtbatchnorm/add:z:0*
T0*
_output_shapes	
:?2
batchnorm/Rsqrt?
batchnorm/mul/ReadVariableOpReadVariableOp%batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype02
batchnorm/mul/ReadVariableOp?
batchnorm/mulMulbatchnorm/Rsqrt:y:0$batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?2
batchnorm/mulw
batchnorm/mul_1Mulinputsbatchnorm/mul:z:0*
T0*(
_output_shapes
:??????????2
batchnorm/mul_1|
batchnorm/mul_2Mulmoments/Squeeze:output:0batchnorm/mul:z:0*
T0*
_output_shapes	
:?2
batchnorm/mul_2?
batchnorm/ReadVariableOpReadVariableOp!batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype02
batchnorm/ReadVariableOp?
batchnorm/subSub batchnorm/ReadVariableOp:value:0batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?2
batchnorm/sub?
batchnorm/add_1AddV2batchnorm/mul_1:z:0batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????2
batchnorm/add_1?
IdentityIdentitybatchnorm/add_1:z:0$^AssignMovingAvg/AssignSubVariableOp^AssignMovingAvg/ReadVariableOp&^AssignMovingAvg_1/AssignSubVariableOp!^AssignMovingAvg_1/ReadVariableOp^batchnorm/ReadVariableOp^batchnorm/mul/ReadVariableOp*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*7
_input_shapes&
$:??????????::::2J
#AssignMovingAvg/AssignSubVariableOp#AssignMovingAvg/AssignSubVariableOp2@
AssignMovingAvg/ReadVariableOpAssignMovingAvg/ReadVariableOp2N
%AssignMovingAvg_1/AssignSubVariableOp%AssignMovingAvg_1/AssignSubVariableOp2D
 AssignMovingAvg_1/ReadVariableOp AssignMovingAvg_1/ReadVariableOp24
batchnorm/ReadVariableOpbatchnorm/ReadVariableOp2<
batchnorm/mul/ReadVariableOpbatchnorm/mul/ReadVariableOp:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_105_layer_call_fn_585772

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_5837672
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????::::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
h
L__inference_max_pooling2d_35_layer_call_and_return_conditional_losses_583064

inputs
identity?
MaxPoolMaxPoolinputs*J
_output_shapes8
6:4????????????????????????????????????*
ksize
*
paddingVALID*
strides
2	
MaxPool?
IdentityIdentityMaxPool:output:0*
T0*J
_output_shapes8
6:4????????????????????????????????????2

Identity"
identityIdentity:output:0*I
_input_shapes8
6:4????????????????????????????????????:r n
J
_output_shapes8
6:4????????????????????????????????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_582912

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??AssignNewValue?AssignNewValue_1?FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*]
_output_shapesK
I:+???????????????????????????:::::*
epsilon%o?:*
exponential_avg_factor%
?#<2
FusedBatchNormV3?
AssignNewValueAssignVariableOp(fusedbatchnormv3_readvariableop_resourceFusedBatchNormV3:batch_mean:0 ^FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*;
_class1
/-loc:@FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02
AssignNewValue?
AssignNewValue_1AssignVariableOp*fusedbatchnormv3_readvariableop_1_resource!FusedBatchNormV3:batch_variance:0"^FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*=
_class3
1/loc:@FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02
AssignNewValue_1?
IdentityIdentityFusedBatchNormV3:y:0^AssignNewValue^AssignNewValue_1 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*A
_output_shapes/
-:+???????????????????????????2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????::::2 
AssignNewValueAssignNewValue2$
AssignNewValue_1AssignNewValue_12B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:i e
A
_output_shapes/
-:+???????????????????????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_583016

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??AssignNewValue?AssignNewValue_1?FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*]
_output_shapesK
I:+???????????????????????????:::::*
epsilon%o?:*
exponential_avg_factor%
?#<2
FusedBatchNormV3?
AssignNewValueAssignVariableOp(fusedbatchnormv3_readvariableop_resourceFusedBatchNormV3:batch_mean:0 ^FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*;
_class1
/-loc:@FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02
AssignNewValue?
AssignNewValue_1AssignVariableOp*fusedbatchnormv3_readvariableop_1_resource!FusedBatchNormV3:batch_variance:0"^FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*=
_class3
1/loc:@FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02
AssignNewValue_1?
IdentityIdentityFusedBatchNormV3:y:0^AssignNewValue^AssignNewValue_1 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*A
_output_shapes/
-:+???????????????????????????2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????::::2 
AssignNewValueAssignNewValue2$
AssignNewValue_1AssignNewValue_12B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:i e
A
_output_shapes/
-:+???????????????????????????
 
_user_specified_nameinputs
?
e
F__inference_dropout_51_layer_call_and_return_conditional_losses_583816

inputs
identity?c
dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *UU??2
dropout/Const{
dropout/MulMulinputsdropout/Const:output:0*
T0*/
_output_shapes
:?????????2
dropout/MulT
dropout/ShapeShapeinputs*
T0*
_output_shapes
:2
dropout/Shape?
$dropout/random_uniform/RandomUniformRandomUniformdropout/Shape:output:0*
T0*/
_output_shapes
:?????????*
dtype02&
$dropout/random_uniform/RandomUniformu
dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *???>2
dropout/GreaterEqual/y?
dropout/GreaterEqualGreaterEqual-dropout/random_uniform/RandomUniform:output:0dropout/GreaterEqual/y:output:0*
T0*/
_output_shapes
:?????????2
dropout/GreaterEqual?
dropout/CastCastdropout/GreaterEqual:z:0*

DstT0*

SrcT0
*/
_output_shapes
:?????????2
dropout/Cast?
dropout/Mul_1Muldropout/Mul:z:0dropout/Cast:y:0*
T0*/
_output_shapes
:?????????2
dropout/Mul_1m
IdentityIdentitydropout/Mul_1:z:0*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
e
F__inference_dropout_53_layer_call_and_return_conditional_losses_584052

inputs
identity?c
dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *UU??2
dropout/Constt
dropout/MulMulinputsdropout/Const:output:0*
T0*(
_output_shapes
:??????????2
dropout/MulT
dropout/ShapeShapeinputs*
T0*
_output_shapes
:2
dropout/Shape?
$dropout/random_uniform/RandomUniformRandomUniformdropout/Shape:output:0*
T0*(
_output_shapes
:??????????*
dtype02&
$dropout/random_uniform/RandomUniformu
dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *???>2
dropout/GreaterEqual/y?
dropout/GreaterEqualGreaterEqual-dropout/random_uniform/RandomUniform:output:0dropout/GreaterEqual/y:output:0*
T0*(
_output_shapes
:??????????2
dropout/GreaterEqual?
dropout/CastCastdropout/GreaterEqual:z:0*

DstT0*

SrcT0
*(
_output_shapes
:??????????2
dropout/Cast{
dropout/Mul_1Muldropout/Mul:z:0dropout/Cast:y:0*
T0*(
_output_shapes
:??????????2
dropout/Mul_1f
IdentityIdentitydropout/Mul_1:z:0*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_103_layer_call_fn_585509

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 *A
_output_shapes/
-:+???????????????????????????<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_5827962
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*A
_output_shapes/
-:+???????????????????????????<2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????<::::22
StatefulPartitionedCallStatefulPartitionedCall:i e
A
_output_shapes/
-:+???????????????????????????<
 
_user_specified_nameinputs
?

*__inference_conv2d_70_layer_call_fn_585541

inputs
unknown
	unknown_0
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_70_layer_call_and_return_conditional_losses_5835892
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????<::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????<
 
_user_specified_nameinputs
?
~
)__inference_dense_51_layer_call_fn_585893

inputs
unknown
	unknown_0
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *M
fHRF
D__inference_dense_51_layer_call_and_return_conditional_losses_5838582
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*/
_input_shapes
:??????????!::22
StatefulPartitionedCallStatefulPartitionedCall:P L
(
_output_shapes
:??????????!
 
_user_specified_nameinputs
?	
?
E__inference_conv2d_69_layer_call_and_return_conditional_losses_585375

inputs"
conv2d_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?Conv2D/ReadVariableOp?
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
:<<*
dtype02
Conv2D/ReadVariableOp?
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????88<*
paddingVALID*
strides
2
Conv2D?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:<*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????88<2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*
T0*/
_output_shapes
:?????????88<2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????<<<::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:?????????<<<
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_104_layer_call_fn_585615

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 *A
_output_shapes/
-:+???????????????????????????*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_5829432
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*A
_output_shapes/
-:+???????????????????????????2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????::::22
StatefulPartitionedCallStatefulPartitionedCall:i e
A
_output_shapes/
-:+???????????????????????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_583542

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:<*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:<*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????88<:<:<:<:<:*
epsilon%o?:*
is_training( 2
FusedBatchNormV3?
IdentityIdentityFusedBatchNormV3:y:0 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*/
_output_shapes
:?????????88<2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????88<::::2B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:W S
/
_output_shapes
:?????????88<
 
_user_specified_nameinputs
?	
?
E__inference_conv2d_70_layer_call_and_return_conditional_losses_583589

inputs"
conv2d_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?Conv2D/ReadVariableOp?
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
:<*
dtype02
Conv2D/ReadVariableOp?
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2
Conv2D?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????<::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:?????????<
 
_user_specified_nameinputs
?
e
F__inference_dropout_52_layer_call_and_return_conditional_losses_583934

inputs
identity?c
dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *UU??2
dropout/Constt
dropout/MulMulinputsdropout/Const:output:0*
T0*(
_output_shapes
:??????????2
dropout/MulT
dropout/ShapeShapeinputs*
T0*
_output_shapes
:2
dropout/Shape?
$dropout/random_uniform/RandomUniformRandomUniformdropout/Shape:output:0*
T0*(
_output_shapes
:??????????*
dtype02&
$dropout/random_uniform/RandomUniformu
dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *???>2
dropout/GreaterEqual/y?
dropout/GreaterEqualGreaterEqual-dropout/random_uniform/RandomUniform:output:0dropout/GreaterEqual/y:output:0*
T0*(
_output_shapes
:??????????2
dropout/GreaterEqual?
dropout/CastCastdropout/GreaterEqual:z:0*

DstT0*

SrcT0
*(
_output_shapes
:??????????2
dropout/Cast{
dropout/Mul_1Muldropout/Mul:z:0dropout/Cast:y:0*
T0*(
_output_shapes
:??????????2
dropout/Mul_1f
IdentityIdentitydropout/Mul_1:z:0*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
d
F__inference_dropout_53_layer_call_and_return_conditional_losses_586151

inputs

identity_1[
IdentityIdentityinputs*
T0*(
_output_shapes
:??????????2

Identityj

Identity_1IdentityIdentity:output:0*
T0*(
_output_shapes
:??????????2

Identity_1"!

identity_1Identity_1:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
e
F__inference_dropout_53_layer_call_and_return_conditional_losses_586146

inputs
identity?c
dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *UU??2
dropout/Constt
dropout/MulMulinputsdropout/Const:output:0*
T0*(
_output_shapes
:??????????2
dropout/MulT
dropout/ShapeShapeinputs*
T0*
_output_shapes
:2
dropout/Shape?
$dropout/random_uniform/RandomUniformRandomUniformdropout/Shape:output:0*
T0*(
_output_shapes
:??????????*
dtype02&
$dropout/random_uniform/RandomUniformu
dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *???>2
dropout/GreaterEqual/y?
dropout/GreaterEqualGreaterEqual-dropout/random_uniform/RandomUniform:output:0dropout/GreaterEqual/y:output:0*
T0*(
_output_shapes
:??????????2
dropout/GreaterEqual?
dropout/CastCastdropout/GreaterEqual:z:0*

DstT0*

SrcT0
*(
_output_shapes
:??????????2
dropout/Cast{
dropout/Mul_1Muldropout/Mul:z:0dropout/Cast:y:0*
T0*(
_output_shapes
:??????????2
dropout/Mul_1f
IdentityIdentitydropout/Mul_1:z:0*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
G
+__inference_dropout_52_layer_call_fn_586012

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_52_layer_call_and_return_conditional_losses_5839392
PartitionedCallm
IdentityIdentityPartitionedCall:output:0*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
?
.__inference_sequential_17_layer_call_fn_584406
conv2d_68_input
unknown
	unknown_0
	unknown_1
	unknown_2
	unknown_3
	unknown_4
	unknown_5
	unknown_6
	unknown_7
	unknown_8
	unknown_9

unknown_10

unknown_11

unknown_12

unknown_13

unknown_14

unknown_15

unknown_16

unknown_17

unknown_18

unknown_19

unknown_20

unknown_21

unknown_22

unknown_23

unknown_24

unknown_25

unknown_26

unknown_27

unknown_28

unknown_29

unknown_30

unknown_31

unknown_32

unknown_33

unknown_34

unknown_35

unknown_36
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallconv2d_68_inputunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
unknown_10
unknown_11
unknown_12
unknown_13
unknown_14
unknown_15
unknown_16
unknown_17
unknown_18
unknown_19
unknown_20
unknown_21
unknown_22
unknown_23
unknown_24
unknown_25
unknown_26
unknown_27
unknown_28
unknown_29
unknown_30
unknown_31
unknown_32
unknown_33
unknown_34
unknown_35
unknown_36*2
Tin+
)2'*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*<
_read_only_resource_inputs
	
 #$%&*0
config_proto 

CPU

GPU2*0J 8? *R
fMRK
I__inference_sequential_17_layer_call_and_return_conditional_losses_5843272
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::22
StatefulPartitionedCallStatefulPartitionedCall:` \
/
_output_shapes
:?????????@@
)
_user_specified_nameconv2d_68_input
?	
?
D__inference_dense_53_layer_call_and_return_conditional_losses_584080

inputs"
matmul_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?MatMul/ReadVariableOp?
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes
:	?*
dtype02
MatMul/ReadVariableOps
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
MatMul?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*/
_input_shapes
:??????????::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
K
/__inference_activation_120_layer_call_fn_585394

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????88<* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_120_layer_call_and_return_conditional_losses_5834972
PartitionedCallt
IdentityIdentityPartitionedCall:output:0*
T0*/
_output_shapes
:?????????88<2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????88<:W S
/
_output_shapes
:?????????88<
 
_user_specified_nameinputs
?	
?
E__inference_conv2d_71_layer_call_and_return_conditional_losses_583701

inputs"
conv2d_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?Conv2D/ReadVariableOp?
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
Conv2D/ReadVariableOp?
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2
Conv2D?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_103_layer_call_fn_585522

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 *A
_output_shapes/
-:+???????????????????????????<*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_5828272
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*A
_output_shapes/
-:+???????????????????????????<2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????<::::22
StatefulPartitionedCallStatefulPartitionedCall:i e
A
_output_shapes/
-:+???????????????????????????<
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_585792

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??AssignNewValue?AssignNewValue_1?FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*]
_output_shapesK
I:+???????????????????????????:::::*
epsilon%o?:*
exponential_avg_factor%
?#<2
FusedBatchNormV3?
AssignNewValueAssignVariableOp(fusedbatchnormv3_readvariableop_resourceFusedBatchNormV3:batch_mean:0 ^FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*;
_class1
/-loc:@FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02
AssignNewValue?
AssignNewValue_1AssignVariableOp*fusedbatchnormv3_readvariableop_1_resource!FusedBatchNormV3:batch_variance:0"^FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*=
_class3
1/loc:@FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02
AssignNewValue_1?
IdentityIdentityFusedBatchNormV3:y:0^AssignNewValue^AssignNewValue_1 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*A
_output_shapes/
-:+???????????????????????????2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????::::2 
AssignNewValueAssignNewValue2$
AssignNewValue_1AssignNewValue_12B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:i e
A
_output_shapes/
-:+???????????????????????????
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_105_layer_call_fn_585836

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 *A
_output_shapes/
-:+???????????????????????????*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_5830472
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*A
_output_shapes/
-:+???????????????????????????2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????::::22
StatefulPartitionedCallStatefulPartitionedCall:i e
A
_output_shapes/
-:+???????????????????????????
 
_user_specified_nameinputs
?
f
J__inference_activation_120_layer_call_and_return_conditional_losses_585389

inputs
identityV
ReluReluinputs*
T0*/
_output_shapes
:?????????88<2
Relun
IdentityIdentityRelu:activations:0*
T0*/
_output_shapes
:?????????88<2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????88<:W S
/
_output_shapes
:?????????88<
 
_user_specified_nameinputs
?
G
+__inference_flatten_35_layer_call_fn_586023

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_flatten_35_layer_call_and_return_conditional_losses_5839582
PartitionedCallm
IdentityIdentityPartitionedCall:output:0*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?	
?
D__inference_dense_52_layer_call_and_return_conditional_losses_583976

inputs"
matmul_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?MatMul/ReadVariableOp?
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource* 
_output_shapes
:
??*
dtype02
MatMul/ReadVariableOpt
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
MatMul?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes	
:?*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*/
_input_shapes
:??????????::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_583430

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:<*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:<*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????<<<:<:<:<:<:*
epsilon%o?:*
is_training( 2
FusedBatchNormV3?
IdentityIdentityFusedBatchNormV3:y:0 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*/
_output_shapes
:?????????<<<2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????<<<::::2B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:W S
/
_output_shapes
:?????????<<<
 
_user_specified_nameinputs
?
f
J__inference_activation_125_layer_call_and_return_conditional_losses_586185

inputs
identityW
SoftmaxSoftmaxinputs*
T0*'
_output_shapes
:?????????2	
Softmaxe
IdentityIdentitySoftmax:softmax:0*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*&
_input_shapes
:?????????:O K
'
_output_shapes
:?????????
 
_user_specified_nameinputs
??
?.
__inference__traced_save_586510
file_prefix/
+savev2_conv2d_68_kernel_read_readvariableop-
)savev2_conv2d_68_bias_read_readvariableop<
8savev2_batch_normalization_102_gamma_read_readvariableop;
7savev2_batch_normalization_102_beta_read_readvariableopB
>savev2_batch_normalization_102_moving_mean_read_readvariableopF
Bsavev2_batch_normalization_102_moving_variance_read_readvariableop/
+savev2_conv2d_69_kernel_read_readvariableop-
)savev2_conv2d_69_bias_read_readvariableop<
8savev2_batch_normalization_103_gamma_read_readvariableop;
7savev2_batch_normalization_103_beta_read_readvariableopB
>savev2_batch_normalization_103_moving_mean_read_readvariableopF
Bsavev2_batch_normalization_103_moving_variance_read_readvariableop/
+savev2_conv2d_70_kernel_read_readvariableop-
)savev2_conv2d_70_bias_read_readvariableop<
8savev2_batch_normalization_104_gamma_read_readvariableop;
7savev2_batch_normalization_104_beta_read_readvariableopB
>savev2_batch_normalization_104_moving_mean_read_readvariableopF
Bsavev2_batch_normalization_104_moving_variance_read_readvariableop/
+savev2_conv2d_71_kernel_read_readvariableop-
)savev2_conv2d_71_bias_read_readvariableop<
8savev2_batch_normalization_105_gamma_read_readvariableop;
7savev2_batch_normalization_105_beta_read_readvariableopB
>savev2_batch_normalization_105_moving_mean_read_readvariableopF
Bsavev2_batch_normalization_105_moving_variance_read_readvariableop.
*savev2_dense_51_kernel_read_readvariableop,
(savev2_dense_51_bias_read_readvariableop<
8savev2_batch_normalization_106_gamma_read_readvariableop;
7savev2_batch_normalization_106_beta_read_readvariableopB
>savev2_batch_normalization_106_moving_mean_read_readvariableopF
Bsavev2_batch_normalization_106_moving_variance_read_readvariableop.
*savev2_dense_52_kernel_read_readvariableop,
(savev2_dense_52_bias_read_readvariableop<
8savev2_batch_normalization_107_gamma_read_readvariableop;
7savev2_batch_normalization_107_beta_read_readvariableopB
>savev2_batch_normalization_107_moving_mean_read_readvariableopF
Bsavev2_batch_normalization_107_moving_variance_read_readvariableop.
*savev2_dense_53_kernel_read_readvariableop,
(savev2_dense_53_bias_read_readvariableop(
$savev2_adam_iter_read_readvariableop	*
&savev2_adam_beta_1_read_readvariableop*
&savev2_adam_beta_2_read_readvariableop)
%savev2_adam_decay_read_readvariableop1
-savev2_adam_learning_rate_read_readvariableop$
 savev2_total_read_readvariableop$
 savev2_count_read_readvariableop&
"savev2_total_1_read_readvariableop&
"savev2_count_1_read_readvariableop6
2savev2_adam_conv2d_68_kernel_m_read_readvariableop4
0savev2_adam_conv2d_68_bias_m_read_readvariableopC
?savev2_adam_batch_normalization_102_gamma_m_read_readvariableopB
>savev2_adam_batch_normalization_102_beta_m_read_readvariableop6
2savev2_adam_conv2d_69_kernel_m_read_readvariableop4
0savev2_adam_conv2d_69_bias_m_read_readvariableopC
?savev2_adam_batch_normalization_103_gamma_m_read_readvariableopB
>savev2_adam_batch_normalization_103_beta_m_read_readvariableop6
2savev2_adam_conv2d_70_kernel_m_read_readvariableop4
0savev2_adam_conv2d_70_bias_m_read_readvariableopC
?savev2_adam_batch_normalization_104_gamma_m_read_readvariableopB
>savev2_adam_batch_normalization_104_beta_m_read_readvariableop6
2savev2_adam_conv2d_71_kernel_m_read_readvariableop4
0savev2_adam_conv2d_71_bias_m_read_readvariableopC
?savev2_adam_batch_normalization_105_gamma_m_read_readvariableopB
>savev2_adam_batch_normalization_105_beta_m_read_readvariableop5
1savev2_adam_dense_51_kernel_m_read_readvariableop3
/savev2_adam_dense_51_bias_m_read_readvariableopC
?savev2_adam_batch_normalization_106_gamma_m_read_readvariableopB
>savev2_adam_batch_normalization_106_beta_m_read_readvariableop5
1savev2_adam_dense_52_kernel_m_read_readvariableop3
/savev2_adam_dense_52_bias_m_read_readvariableopC
?savev2_adam_batch_normalization_107_gamma_m_read_readvariableopB
>savev2_adam_batch_normalization_107_beta_m_read_readvariableop5
1savev2_adam_dense_53_kernel_m_read_readvariableop3
/savev2_adam_dense_53_bias_m_read_readvariableop6
2savev2_adam_conv2d_68_kernel_v_read_readvariableop4
0savev2_adam_conv2d_68_bias_v_read_readvariableopC
?savev2_adam_batch_normalization_102_gamma_v_read_readvariableopB
>savev2_adam_batch_normalization_102_beta_v_read_readvariableop6
2savev2_adam_conv2d_69_kernel_v_read_readvariableop4
0savev2_adam_conv2d_69_bias_v_read_readvariableopC
?savev2_adam_batch_normalization_103_gamma_v_read_readvariableopB
>savev2_adam_batch_normalization_103_beta_v_read_readvariableop6
2savev2_adam_conv2d_70_kernel_v_read_readvariableop4
0savev2_adam_conv2d_70_bias_v_read_readvariableopC
?savev2_adam_batch_normalization_104_gamma_v_read_readvariableopB
>savev2_adam_batch_normalization_104_beta_v_read_readvariableop6
2savev2_adam_conv2d_71_kernel_v_read_readvariableop4
0savev2_adam_conv2d_71_bias_v_read_readvariableopC
?savev2_adam_batch_normalization_105_gamma_v_read_readvariableopB
>savev2_adam_batch_normalization_105_beta_v_read_readvariableop5
1savev2_adam_dense_51_kernel_v_read_readvariableop3
/savev2_adam_dense_51_bias_v_read_readvariableopC
?savev2_adam_batch_normalization_106_gamma_v_read_readvariableopB
>savev2_adam_batch_normalization_106_beta_v_read_readvariableop5
1savev2_adam_dense_52_kernel_v_read_readvariableop3
/savev2_adam_dense_52_bias_v_read_readvariableopC
?savev2_adam_batch_normalization_107_gamma_v_read_readvariableopB
>savev2_adam_batch_normalization_107_beta_v_read_readvariableop5
1savev2_adam_dense_53_kernel_v_read_readvariableop3
/savev2_adam_dense_53_bias_v_read_readvariableop
savev2_const

identity_1??MergeV2Checkpoints?
StaticRegexFullMatchStaticRegexFullMatchfile_prefix"/device:CPU:**
_output_shapes
: *
pattern
^s3://.*2
StaticRegexFullMatchc
ConstConst"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B.part2
Constl
Const_1Const"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B
_temp/part2	
Const_1?
SelectSelectStaticRegexFullMatch:output:0Const:output:0Const_1:output:0"/device:CPU:**
T0*
_output_shapes
: 2
Selectt

StringJoin
StringJoinfile_prefixSelect:output:0"/device:CPU:**
N*
_output_shapes
: 2

StringJoinZ

num_shardsConst*
_output_shapes
: *
dtype0*
value	B :2

num_shards
ShardedFilename/shardConst"/device:CPU:0*
_output_shapes
: *
dtype0*
value	B : 2
ShardedFilename/shard?
ShardedFilenameShardedFilenameStringJoin:output:0ShardedFilename/shard:output:0num_shards:output:0"/device:CPU:0*
_output_shapes
: 2
ShardedFilename?7
SaveV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:d*
dtype0*?6
value?6B?6dB6layer_with_weights-0/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-0/bias/.ATTRIBUTES/VARIABLE_VALUEB5layer_with_weights-1/gamma/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-1/beta/.ATTRIBUTES/VARIABLE_VALUEB;layer_with_weights-1/moving_mean/.ATTRIBUTES/VARIABLE_VALUEB?layer_with_weights-1/moving_variance/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-2/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-2/bias/.ATTRIBUTES/VARIABLE_VALUEB5layer_with_weights-3/gamma/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-3/beta/.ATTRIBUTES/VARIABLE_VALUEB;layer_with_weights-3/moving_mean/.ATTRIBUTES/VARIABLE_VALUEB?layer_with_weights-3/moving_variance/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-4/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-4/bias/.ATTRIBUTES/VARIABLE_VALUEB5layer_with_weights-5/gamma/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-5/beta/.ATTRIBUTES/VARIABLE_VALUEB;layer_with_weights-5/moving_mean/.ATTRIBUTES/VARIABLE_VALUEB?layer_with_weights-5/moving_variance/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-6/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-6/bias/.ATTRIBUTES/VARIABLE_VALUEB5layer_with_weights-7/gamma/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-7/beta/.ATTRIBUTES/VARIABLE_VALUEB;layer_with_weights-7/moving_mean/.ATTRIBUTES/VARIABLE_VALUEB?layer_with_weights-7/moving_variance/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-8/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-8/bias/.ATTRIBUTES/VARIABLE_VALUEB5layer_with_weights-9/gamma/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-9/beta/.ATTRIBUTES/VARIABLE_VALUEB;layer_with_weights-9/moving_mean/.ATTRIBUTES/VARIABLE_VALUEB?layer_with_weights-9/moving_variance/.ATTRIBUTES/VARIABLE_VALUEB7layer_with_weights-10/kernel/.ATTRIBUTES/VARIABLE_VALUEB5layer_with_weights-10/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-11/gamma/.ATTRIBUTES/VARIABLE_VALUEB5layer_with_weights-11/beta/.ATTRIBUTES/VARIABLE_VALUEB<layer_with_weights-11/moving_mean/.ATTRIBUTES/VARIABLE_VALUEB@layer_with_weights-11/moving_variance/.ATTRIBUTES/VARIABLE_VALUEB7layer_with_weights-12/kernel/.ATTRIBUTES/VARIABLE_VALUEB5layer_with_weights-12/bias/.ATTRIBUTES/VARIABLE_VALUEB)optimizer/iter/.ATTRIBUTES/VARIABLE_VALUEB+optimizer/beta_1/.ATTRIBUTES/VARIABLE_VALUEB+optimizer/beta_2/.ATTRIBUTES/VARIABLE_VALUEB*optimizer/decay/.ATTRIBUTES/VARIABLE_VALUEB2optimizer/learning_rate/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/count/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/1/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/1/count/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-1/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-1/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-3/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-3/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-4/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-4/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-5/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-5/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-6/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-6/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-7/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-7/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-8/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-8/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-9/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-9/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBSlayer_with_weights-10/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-10/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-11/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-11/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBSlayer_with_weights-12/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-12/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-1/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-1/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-3/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-3/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-4/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-4/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-5/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-5/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-6/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-6/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-7/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-7/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-8/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-8/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-9/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-9/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBSlayer_with_weights-10/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-10/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-11/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-11/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBSlayer_with_weights-12/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-12/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH2
SaveV2/tensor_names?
SaveV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:d*
dtype0*?
value?B?dB B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B 2
SaveV2/shape_and_slices?,
SaveV2SaveV2ShardedFilename:filename:0SaveV2/tensor_names:output:0 SaveV2/shape_and_slices:output:0+savev2_conv2d_68_kernel_read_readvariableop)savev2_conv2d_68_bias_read_readvariableop8savev2_batch_normalization_102_gamma_read_readvariableop7savev2_batch_normalization_102_beta_read_readvariableop>savev2_batch_normalization_102_moving_mean_read_readvariableopBsavev2_batch_normalization_102_moving_variance_read_readvariableop+savev2_conv2d_69_kernel_read_readvariableop)savev2_conv2d_69_bias_read_readvariableop8savev2_batch_normalization_103_gamma_read_readvariableop7savev2_batch_normalization_103_beta_read_readvariableop>savev2_batch_normalization_103_moving_mean_read_readvariableopBsavev2_batch_normalization_103_moving_variance_read_readvariableop+savev2_conv2d_70_kernel_read_readvariableop)savev2_conv2d_70_bias_read_readvariableop8savev2_batch_normalization_104_gamma_read_readvariableop7savev2_batch_normalization_104_beta_read_readvariableop>savev2_batch_normalization_104_moving_mean_read_readvariableopBsavev2_batch_normalization_104_moving_variance_read_readvariableop+savev2_conv2d_71_kernel_read_readvariableop)savev2_conv2d_71_bias_read_readvariableop8savev2_batch_normalization_105_gamma_read_readvariableop7savev2_batch_normalization_105_beta_read_readvariableop>savev2_batch_normalization_105_moving_mean_read_readvariableopBsavev2_batch_normalization_105_moving_variance_read_readvariableop*savev2_dense_51_kernel_read_readvariableop(savev2_dense_51_bias_read_readvariableop8savev2_batch_normalization_106_gamma_read_readvariableop7savev2_batch_normalization_106_beta_read_readvariableop>savev2_batch_normalization_106_moving_mean_read_readvariableopBsavev2_batch_normalization_106_moving_variance_read_readvariableop*savev2_dense_52_kernel_read_readvariableop(savev2_dense_52_bias_read_readvariableop8savev2_batch_normalization_107_gamma_read_readvariableop7savev2_batch_normalization_107_beta_read_readvariableop>savev2_batch_normalization_107_moving_mean_read_readvariableopBsavev2_batch_normalization_107_moving_variance_read_readvariableop*savev2_dense_53_kernel_read_readvariableop(savev2_dense_53_bias_read_readvariableop$savev2_adam_iter_read_readvariableop&savev2_adam_beta_1_read_readvariableop&savev2_adam_beta_2_read_readvariableop%savev2_adam_decay_read_readvariableop-savev2_adam_learning_rate_read_readvariableop savev2_total_read_readvariableop savev2_count_read_readvariableop"savev2_total_1_read_readvariableop"savev2_count_1_read_readvariableop2savev2_adam_conv2d_68_kernel_m_read_readvariableop0savev2_adam_conv2d_68_bias_m_read_readvariableop?savev2_adam_batch_normalization_102_gamma_m_read_readvariableop>savev2_adam_batch_normalization_102_beta_m_read_readvariableop2savev2_adam_conv2d_69_kernel_m_read_readvariableop0savev2_adam_conv2d_69_bias_m_read_readvariableop?savev2_adam_batch_normalization_103_gamma_m_read_readvariableop>savev2_adam_batch_normalization_103_beta_m_read_readvariableop2savev2_adam_conv2d_70_kernel_m_read_readvariableop0savev2_adam_conv2d_70_bias_m_read_readvariableop?savev2_adam_batch_normalization_104_gamma_m_read_readvariableop>savev2_adam_batch_normalization_104_beta_m_read_readvariableop2savev2_adam_conv2d_71_kernel_m_read_readvariableop0savev2_adam_conv2d_71_bias_m_read_readvariableop?savev2_adam_batch_normalization_105_gamma_m_read_readvariableop>savev2_adam_batch_normalization_105_beta_m_read_readvariableop1savev2_adam_dense_51_kernel_m_read_readvariableop/savev2_adam_dense_51_bias_m_read_readvariableop?savev2_adam_batch_normalization_106_gamma_m_read_readvariableop>savev2_adam_batch_normalization_106_beta_m_read_readvariableop1savev2_adam_dense_52_kernel_m_read_readvariableop/savev2_adam_dense_52_bias_m_read_readvariableop?savev2_adam_batch_normalization_107_gamma_m_read_readvariableop>savev2_adam_batch_normalization_107_beta_m_read_readvariableop1savev2_adam_dense_53_kernel_m_read_readvariableop/savev2_adam_dense_53_bias_m_read_readvariableop2savev2_adam_conv2d_68_kernel_v_read_readvariableop0savev2_adam_conv2d_68_bias_v_read_readvariableop?savev2_adam_batch_normalization_102_gamma_v_read_readvariableop>savev2_adam_batch_normalization_102_beta_v_read_readvariableop2savev2_adam_conv2d_69_kernel_v_read_readvariableop0savev2_adam_conv2d_69_bias_v_read_readvariableop?savev2_adam_batch_normalization_103_gamma_v_read_readvariableop>savev2_adam_batch_normalization_103_beta_v_read_readvariableop2savev2_adam_conv2d_70_kernel_v_read_readvariableop0savev2_adam_conv2d_70_bias_v_read_readvariableop?savev2_adam_batch_normalization_104_gamma_v_read_readvariableop>savev2_adam_batch_normalization_104_beta_v_read_readvariableop2savev2_adam_conv2d_71_kernel_v_read_readvariableop0savev2_adam_conv2d_71_bias_v_read_readvariableop?savev2_adam_batch_normalization_105_gamma_v_read_readvariableop>savev2_adam_batch_normalization_105_beta_v_read_readvariableop1savev2_adam_dense_51_kernel_v_read_readvariableop/savev2_adam_dense_51_bias_v_read_readvariableop?savev2_adam_batch_normalization_106_gamma_v_read_readvariableop>savev2_adam_batch_normalization_106_beta_v_read_readvariableop1savev2_adam_dense_52_kernel_v_read_readvariableop/savev2_adam_dense_52_bias_v_read_readvariableop?savev2_adam_batch_normalization_107_gamma_v_read_readvariableop>savev2_adam_batch_normalization_107_beta_v_read_readvariableop1savev2_adam_dense_53_kernel_v_read_readvariableop/savev2_adam_dense_53_bias_v_read_readvariableopsavev2_const"/device:CPU:0*
_output_shapes
 *r
dtypesh
f2d	2
SaveV2?
&MergeV2Checkpoints/checkpoint_prefixesPackShardedFilename:filename:0^SaveV2"/device:CPU:0*
N*
T0*
_output_shapes
:2(
&MergeV2Checkpoints/checkpoint_prefixes?
MergeV2CheckpointsMergeV2Checkpoints/MergeV2Checkpoints/checkpoint_prefixes:output:0file_prefix"/device:CPU:0*
_output_shapes
 2
MergeV2Checkpointsr
IdentityIdentityfile_prefix^MergeV2Checkpoints"/device:CPU:0*
T0*
_output_shapes
: 2

Identitym

Identity_1IdentityIdentity:output:0^MergeV2Checkpoints*
T0*
_output_shapes
: 2

Identity_1"!

identity_1Identity_1:output:0*?
_input_shapes?
?: :<:<:<:<:<:<:<<:<:<:<:<:<:<::::::::::::
?!?:?:?:?:?:?:
??:?:?:?:?:?:	?:: : : : : : : : : :<:<:<:<:<<:<:<:<:<::::::::
?!?:?:?:?:
??:?:?:?:	?::<:<:<:<:<<:<:<:<:<::::::::
?!?:?:?:?:
??:?:?:?:	?:: 2(
MergeV2CheckpointsMergeV2Checkpoints:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix:,(
&
_output_shapes
:<: 

_output_shapes
:<: 

_output_shapes
:<: 

_output_shapes
:<: 

_output_shapes
:<: 

_output_shapes
:<:,(
&
_output_shapes
:<<: 

_output_shapes
:<: 	

_output_shapes
:<: 


_output_shapes
:<: 

_output_shapes
:<: 

_output_shapes
:<:,(
&
_output_shapes
:<: 

_output_shapes
:: 

_output_shapes
:: 

_output_shapes
:: 

_output_shapes
:: 

_output_shapes
::,(
&
_output_shapes
:: 

_output_shapes
:: 

_output_shapes
:: 

_output_shapes
:: 

_output_shapes
:: 

_output_shapes
::&"
 
_output_shapes
:
?!?:!

_output_shapes	
:?:!

_output_shapes	
:?:!

_output_shapes	
:?:!

_output_shapes	
:?:!

_output_shapes	
:?:&"
 
_output_shapes
:
??:! 

_output_shapes	
:?:!!

_output_shapes	
:?:!"

_output_shapes	
:?:!#

_output_shapes	
:?:!$

_output_shapes	
:?:%%!

_output_shapes
:	?: &

_output_shapes
::'

_output_shapes
: :(

_output_shapes
: :)

_output_shapes
: :*

_output_shapes
: :+

_output_shapes
: :,

_output_shapes
: :-

_output_shapes
: :.

_output_shapes
: :/

_output_shapes
: :,0(
&
_output_shapes
:<: 1

_output_shapes
:<: 2

_output_shapes
:<: 3

_output_shapes
:<:,4(
&
_output_shapes
:<<: 5

_output_shapes
:<: 6

_output_shapes
:<: 7

_output_shapes
:<:,8(
&
_output_shapes
:<: 9

_output_shapes
:: :

_output_shapes
:: ;

_output_shapes
::,<(
&
_output_shapes
:: =

_output_shapes
:: >

_output_shapes
:: ?

_output_shapes
::&@"
 
_output_shapes
:
?!?:!A

_output_shapes	
:?:!B

_output_shapes	
:?:!C

_output_shapes	
:?:&D"
 
_output_shapes
:
??:!E

_output_shapes	
:?:!F

_output_shapes	
:?:!G

_output_shapes	
:?:%H!

_output_shapes
:	?: I

_output_shapes
::,J(
&
_output_shapes
:<: K

_output_shapes
:<: L

_output_shapes
:<: M

_output_shapes
:<:,N(
&
_output_shapes
:<<: O

_output_shapes
:<: P

_output_shapes
:<: Q

_output_shapes
:<:,R(
&
_output_shapes
:<: S

_output_shapes
:: T

_output_shapes
:: U

_output_shapes
::,V(
&
_output_shapes
:: W

_output_shapes
:: X

_output_shapes
:: Y

_output_shapes
::&Z"
 
_output_shapes
:
?!?:![

_output_shapes	
:?:!\

_output_shapes	
:?:!]

_output_shapes	
:?:&^"
 
_output_shapes
:
??:!_

_output_shapes	
:?:!`

_output_shapes	
:?:!a

_output_shapes	
:?:%b!

_output_shapes
:	?: c

_output_shapes
::d

_output_shapes
: 
?	
?
E__inference_conv2d_69_layer_call_and_return_conditional_losses_583476

inputs"
conv2d_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?Conv2D/ReadVariableOp?
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
:<<*
dtype02
Conv2D/ReadVariableOp?
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????88<*
paddingVALID*
strides
2
Conv2D?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:<*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????88<2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*
T0*/
_output_shapes
:?????????88<2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????<<<::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:?????????<<<
 
_user_specified_nameinputs
??
?
I__inference_sequential_17_layer_call_and_return_conditional_losses_584327

inputs
conv2d_68_584223
conv2d_68_584225"
batch_normalization_102_584229"
batch_normalization_102_584231"
batch_normalization_102_584233"
batch_normalization_102_584235
conv2d_69_584238
conv2d_69_584240"
batch_normalization_103_584244"
batch_normalization_103_584246"
batch_normalization_103_584248"
batch_normalization_103_584250
conv2d_70_584254
conv2d_70_584256"
batch_normalization_104_584260"
batch_normalization_104_584262"
batch_normalization_104_584264"
batch_normalization_104_584266
conv2d_71_584269
conv2d_71_584271"
batch_normalization_105_584275"
batch_normalization_105_584277"
batch_normalization_105_584279"
batch_normalization_105_584281
dense_51_584287
dense_51_584289"
batch_normalization_106_584293"
batch_normalization_106_584295"
batch_normalization_106_584297"
batch_normalization_106_584299
dense_52_584304
dense_52_584306"
batch_normalization_107_584310"
batch_normalization_107_584312"
batch_normalization_107_584314"
batch_normalization_107_584316
dense_53_584320
dense_53_584322
identity??/batch_normalization_102/StatefulPartitionedCall?/batch_normalization_103/StatefulPartitionedCall?/batch_normalization_104/StatefulPartitionedCall?/batch_normalization_105/StatefulPartitionedCall?/batch_normalization_106/StatefulPartitionedCall?/batch_normalization_107/StatefulPartitionedCall?!conv2d_68/StatefulPartitionedCall?!conv2d_69/StatefulPartitionedCall?!conv2d_70/StatefulPartitionedCall?!conv2d_71/StatefulPartitionedCall? dense_51/StatefulPartitionedCall? dense_52/StatefulPartitionedCall? dense_53/StatefulPartitionedCall?"dropout_51/StatefulPartitionedCall?"dropout_52/StatefulPartitionedCall?"dropout_53/StatefulPartitionedCall?
!conv2d_68/StatefulPartitionedCallStatefulPartitionedCallinputsconv2d_68_584223conv2d_68_584225*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<<<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_68_layer_call_and_return_conditional_losses_5833642#
!conv2d_68/StatefulPartitionedCall?
activation_119/PartitionedCallPartitionedCall*conv2d_68/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<<<* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_119_layer_call_and_return_conditional_losses_5833852 
activation_119/PartitionedCall?
/batch_normalization_102/StatefulPartitionedCallStatefulPartitionedCall'activation_119/PartitionedCall:output:0batch_normalization_102_584229batch_normalization_102_584231batch_normalization_102_584233batch_normalization_102_584235*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<<<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_58341221
/batch_normalization_102/StatefulPartitionedCall?
!conv2d_69/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_102/StatefulPartitionedCall:output:0conv2d_69_584238conv2d_69_584240*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????88<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_69_layer_call_and_return_conditional_losses_5834762#
!conv2d_69/StatefulPartitionedCall?
activation_120/PartitionedCallPartitionedCall*conv2d_69/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????88<* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_120_layer_call_and_return_conditional_losses_5834972 
activation_120/PartitionedCall?
/batch_normalization_103/StatefulPartitionedCallStatefulPartitionedCall'activation_120/PartitionedCall:output:0batch_normalization_103_584244batch_normalization_103_584246batch_normalization_103_584248batch_normalization_103_584250*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????88<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_58352421
/batch_normalization_103/StatefulPartitionedCall?
 max_pooling2d_34/PartitionedCallPartitionedCall8batch_normalization_103/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *U
fPRN
L__inference_max_pooling2d_34_layer_call_and_return_conditional_losses_5828442"
 max_pooling2d_34/PartitionedCall?
!conv2d_70/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_34/PartitionedCall:output:0conv2d_70_584254conv2d_70_584256*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_70_layer_call_and_return_conditional_losses_5835892#
!conv2d_70/StatefulPartitionedCall?
activation_121/PartitionedCallPartitionedCall*conv2d_70/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_121_layer_call_and_return_conditional_losses_5836102 
activation_121/PartitionedCall?
/batch_normalization_104/StatefulPartitionedCallStatefulPartitionedCall'activation_121/PartitionedCall:output:0batch_normalization_104_584260batch_normalization_104_584262batch_normalization_104_584264batch_normalization_104_584266*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_58363721
/batch_normalization_104/StatefulPartitionedCall?
!conv2d_71/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_104/StatefulPartitionedCall:output:0conv2d_71_584269conv2d_71_584271*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_71_layer_call_and_return_conditional_losses_5837012#
!conv2d_71/StatefulPartitionedCall?
activation_122/PartitionedCallPartitionedCall*conv2d_71/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_122_layer_call_and_return_conditional_losses_5837222 
activation_122/PartitionedCall?
/batch_normalization_105/StatefulPartitionedCallStatefulPartitionedCall'activation_122/PartitionedCall:output:0batch_normalization_105_584275batch_normalization_105_584277batch_normalization_105_584279batch_normalization_105_584281*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_58374921
/batch_normalization_105/StatefulPartitionedCall?
 max_pooling2d_35/PartitionedCallPartitionedCall8batch_normalization_105/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *U
fPRN
L__inference_max_pooling2d_35_layer_call_and_return_conditional_losses_5830642"
 max_pooling2d_35/PartitionedCall?
"dropout_51/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_35/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_51_layer_call_and_return_conditional_losses_5838162$
"dropout_51/StatefulPartitionedCall?
flatten_34/PartitionedCallPartitionedCall+dropout_51/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????!* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_flatten_34_layer_call_and_return_conditional_losses_5838402
flatten_34/PartitionedCall?
 dense_51/StatefulPartitionedCallStatefulPartitionedCall#flatten_34/PartitionedCall:output:0dense_51_584287dense_51_584289*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *M
fHRF
D__inference_dense_51_layer_call_and_return_conditional_losses_5838582"
 dense_51/StatefulPartitionedCall?
activation_123/PartitionedCallPartitionedCall)dense_51/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_123_layer_call_and_return_conditional_losses_5838792 
activation_123/PartitionedCall?
/batch_normalization_106/StatefulPartitionedCallStatefulPartitionedCall'activation_123/PartitionedCall:output:0batch_normalization_106_584293batch_normalization_106_584295batch_normalization_106_584297batch_normalization_106_584299*
Tin	
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_106_layer_call_and_return_conditional_losses_58316621
/batch_normalization_106/StatefulPartitionedCall?
"dropout_52/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_106/StatefulPartitionedCall:output:0#^dropout_51/StatefulPartitionedCall*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_52_layer_call_and_return_conditional_losses_5839342$
"dropout_52/StatefulPartitionedCall?
flatten_35/PartitionedCallPartitionedCall+dropout_52/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_flatten_35_layer_call_and_return_conditional_losses_5839582
flatten_35/PartitionedCall?
 dense_52/StatefulPartitionedCallStatefulPartitionedCall#flatten_35/PartitionedCall:output:0dense_52_584304dense_52_584306*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *M
fHRF
D__inference_dense_52_layer_call_and_return_conditional_losses_5839762"
 dense_52/StatefulPartitionedCall?
activation_124/PartitionedCallPartitionedCall)dense_52/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_124_layer_call_and_return_conditional_losses_5839972 
activation_124/PartitionedCall?
/batch_normalization_107/StatefulPartitionedCallStatefulPartitionedCall'activation_124/PartitionedCall:output:0batch_normalization_107_584310batch_normalization_107_584312batch_normalization_107_584314batch_normalization_107_584316*
Tin	
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_107_layer_call_and_return_conditional_losses_58330621
/batch_normalization_107/StatefulPartitionedCall?
"dropout_53/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_107/StatefulPartitionedCall:output:0#^dropout_52/StatefulPartitionedCall*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_53_layer_call_and_return_conditional_losses_5840522$
"dropout_53/StatefulPartitionedCall?
 dense_53/StatefulPartitionedCallStatefulPartitionedCall+dropout_53/StatefulPartitionedCall:output:0dense_53_584320dense_53_584322*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *M
fHRF
D__inference_dense_53_layer_call_and_return_conditional_losses_5840802"
 dense_53/StatefulPartitionedCall?
activation_125/PartitionedCallPartitionedCall)dense_53/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_125_layer_call_and_return_conditional_losses_5841012 
activation_125/PartitionedCall?
IdentityIdentity'activation_125/PartitionedCall:output:00^batch_normalization_102/StatefulPartitionedCall0^batch_normalization_103/StatefulPartitionedCall0^batch_normalization_104/StatefulPartitionedCall0^batch_normalization_105/StatefulPartitionedCall0^batch_normalization_106/StatefulPartitionedCall0^batch_normalization_107/StatefulPartitionedCall"^conv2d_68/StatefulPartitionedCall"^conv2d_69/StatefulPartitionedCall"^conv2d_70/StatefulPartitionedCall"^conv2d_71/StatefulPartitionedCall!^dense_51/StatefulPartitionedCall!^dense_52/StatefulPartitionedCall!^dense_53/StatefulPartitionedCall#^dropout_51/StatefulPartitionedCall#^dropout_52/StatefulPartitionedCall#^dropout_53/StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::2b
/batch_normalization_102/StatefulPartitionedCall/batch_normalization_102/StatefulPartitionedCall2b
/batch_normalization_103/StatefulPartitionedCall/batch_normalization_103/StatefulPartitionedCall2b
/batch_normalization_104/StatefulPartitionedCall/batch_normalization_104/StatefulPartitionedCall2b
/batch_normalization_105/StatefulPartitionedCall/batch_normalization_105/StatefulPartitionedCall2b
/batch_normalization_106/StatefulPartitionedCall/batch_normalization_106/StatefulPartitionedCall2b
/batch_normalization_107/StatefulPartitionedCall/batch_normalization_107/StatefulPartitionedCall2F
!conv2d_68/StatefulPartitionedCall!conv2d_68/StatefulPartitionedCall2F
!conv2d_69/StatefulPartitionedCall!conv2d_69/StatefulPartitionedCall2F
!conv2d_70/StatefulPartitionedCall!conv2d_70/StatefulPartitionedCall2F
!conv2d_71/StatefulPartitionedCall!conv2d_71/StatefulPartitionedCall2D
 dense_51/StatefulPartitionedCall dense_51/StatefulPartitionedCall2D
 dense_52/StatefulPartitionedCall dense_52/StatefulPartitionedCall2D
 dense_53/StatefulPartitionedCall dense_53/StatefulPartitionedCall2H
"dropout_51/StatefulPartitionedCall"dropout_51/StatefulPartitionedCall2H
"dropout_52/StatefulPartitionedCall"dropout_52/StatefulPartitionedCall2H
"dropout_53/StatefulPartitionedCall"dropout_53/StatefulPartitionedCall:W S
/
_output_shapes
:?????????@@
 
_user_specified_nameinputs
?
d
+__inference_dropout_52_layer_call_fn_586007

inputs
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_52_layer_call_and_return_conditional_losses_5839342
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????22
StatefulPartitionedCallStatefulPartitionedCall:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_583412

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??AssignNewValue?AssignNewValue_1?FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:<*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:<*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????<<<:<:<:<:<:*
epsilon%o?:*
exponential_avg_factor%
?#<2
FusedBatchNormV3?
AssignNewValueAssignVariableOp(fusedbatchnormv3_readvariableop_resourceFusedBatchNormV3:batch_mean:0 ^FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*;
_class1
/-loc:@FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02
AssignNewValue?
AssignNewValue_1AssignVariableOp*fusedbatchnormv3_readvariableop_1_resource!FusedBatchNormV3:batch_variance:0"^FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*=
_class3
1/loc:@FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02
AssignNewValue_1?
IdentityIdentityFusedBatchNormV3:y:0^AssignNewValue^AssignNewValue_1 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*/
_output_shapes
:?????????<<<2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????<<<::::2 
AssignNewValueAssignNewValue2$
AssignNewValue_1AssignNewValue_12B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:W S
/
_output_shapes
:?????????<<<
 
_user_specified_nameinputs
?
f
J__inference_activation_120_layer_call_and_return_conditional_losses_583497

inputs
identityV
ReluReluinputs*
T0*/
_output_shapes
:?????????88<2
Relun
IdentityIdentityRelu:activations:0*
T0*/
_output_shapes
:?????????88<2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????88<:W S
/
_output_shapes
:?????????88<
 
_user_specified_nameinputs
??
?
I__inference_sequential_17_layer_call_and_return_conditional_losses_584110
conv2d_68_input
conv2d_68_583375
conv2d_68_583377"
batch_normalization_102_583457"
batch_normalization_102_583459"
batch_normalization_102_583461"
batch_normalization_102_583463
conv2d_69_583487
conv2d_69_583489"
batch_normalization_103_583569"
batch_normalization_103_583571"
batch_normalization_103_583573"
batch_normalization_103_583575
conv2d_70_583600
conv2d_70_583602"
batch_normalization_104_583682"
batch_normalization_104_583684"
batch_normalization_104_583686"
batch_normalization_104_583688
conv2d_71_583712
conv2d_71_583714"
batch_normalization_105_583794"
batch_normalization_105_583796"
batch_normalization_105_583798"
batch_normalization_105_583800
dense_51_583869
dense_51_583871"
batch_normalization_106_583913"
batch_normalization_106_583915"
batch_normalization_106_583917"
batch_normalization_106_583919
dense_52_583987
dense_52_583989"
batch_normalization_107_584031"
batch_normalization_107_584033"
batch_normalization_107_584035"
batch_normalization_107_584037
dense_53_584091
dense_53_584093
identity??/batch_normalization_102/StatefulPartitionedCall?/batch_normalization_103/StatefulPartitionedCall?/batch_normalization_104/StatefulPartitionedCall?/batch_normalization_105/StatefulPartitionedCall?/batch_normalization_106/StatefulPartitionedCall?/batch_normalization_107/StatefulPartitionedCall?!conv2d_68/StatefulPartitionedCall?!conv2d_69/StatefulPartitionedCall?!conv2d_70/StatefulPartitionedCall?!conv2d_71/StatefulPartitionedCall? dense_51/StatefulPartitionedCall? dense_52/StatefulPartitionedCall? dense_53/StatefulPartitionedCall?"dropout_51/StatefulPartitionedCall?"dropout_52/StatefulPartitionedCall?"dropout_53/StatefulPartitionedCall?
!conv2d_68/StatefulPartitionedCallStatefulPartitionedCallconv2d_68_inputconv2d_68_583375conv2d_68_583377*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<<<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_68_layer_call_and_return_conditional_losses_5833642#
!conv2d_68/StatefulPartitionedCall?
activation_119/PartitionedCallPartitionedCall*conv2d_68/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<<<* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_119_layer_call_and_return_conditional_losses_5833852 
activation_119/PartitionedCall?
/batch_normalization_102/StatefulPartitionedCallStatefulPartitionedCall'activation_119/PartitionedCall:output:0batch_normalization_102_583457batch_normalization_102_583459batch_normalization_102_583461batch_normalization_102_583463*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<<<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_58341221
/batch_normalization_102/StatefulPartitionedCall?
!conv2d_69/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_102/StatefulPartitionedCall:output:0conv2d_69_583487conv2d_69_583489*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????88<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_69_layer_call_and_return_conditional_losses_5834762#
!conv2d_69/StatefulPartitionedCall?
activation_120/PartitionedCallPartitionedCall*conv2d_69/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????88<* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_120_layer_call_and_return_conditional_losses_5834972 
activation_120/PartitionedCall?
/batch_normalization_103/StatefulPartitionedCallStatefulPartitionedCall'activation_120/PartitionedCall:output:0batch_normalization_103_583569batch_normalization_103_583571batch_normalization_103_583573batch_normalization_103_583575*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????88<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_58352421
/batch_normalization_103/StatefulPartitionedCall?
 max_pooling2d_34/PartitionedCallPartitionedCall8batch_normalization_103/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *U
fPRN
L__inference_max_pooling2d_34_layer_call_and_return_conditional_losses_5828442"
 max_pooling2d_34/PartitionedCall?
!conv2d_70/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_34/PartitionedCall:output:0conv2d_70_583600conv2d_70_583602*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_70_layer_call_and_return_conditional_losses_5835892#
!conv2d_70/StatefulPartitionedCall?
activation_121/PartitionedCallPartitionedCall*conv2d_70/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_121_layer_call_and_return_conditional_losses_5836102 
activation_121/PartitionedCall?
/batch_normalization_104/StatefulPartitionedCallStatefulPartitionedCall'activation_121/PartitionedCall:output:0batch_normalization_104_583682batch_normalization_104_583684batch_normalization_104_583686batch_normalization_104_583688*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_58363721
/batch_normalization_104/StatefulPartitionedCall?
!conv2d_71/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_104/StatefulPartitionedCall:output:0conv2d_71_583712conv2d_71_583714*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_71_layer_call_and_return_conditional_losses_5837012#
!conv2d_71/StatefulPartitionedCall?
activation_122/PartitionedCallPartitionedCall*conv2d_71/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_122_layer_call_and_return_conditional_losses_5837222 
activation_122/PartitionedCall?
/batch_normalization_105/StatefulPartitionedCallStatefulPartitionedCall'activation_122/PartitionedCall:output:0batch_normalization_105_583794batch_normalization_105_583796batch_normalization_105_583798batch_normalization_105_583800*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_58374921
/batch_normalization_105/StatefulPartitionedCall?
 max_pooling2d_35/PartitionedCallPartitionedCall8batch_normalization_105/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *U
fPRN
L__inference_max_pooling2d_35_layer_call_and_return_conditional_losses_5830642"
 max_pooling2d_35/PartitionedCall?
"dropout_51/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_35/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_51_layer_call_and_return_conditional_losses_5838162$
"dropout_51/StatefulPartitionedCall?
flatten_34/PartitionedCallPartitionedCall+dropout_51/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????!* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_flatten_34_layer_call_and_return_conditional_losses_5838402
flatten_34/PartitionedCall?
 dense_51/StatefulPartitionedCallStatefulPartitionedCall#flatten_34/PartitionedCall:output:0dense_51_583869dense_51_583871*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *M
fHRF
D__inference_dense_51_layer_call_and_return_conditional_losses_5838582"
 dense_51/StatefulPartitionedCall?
activation_123/PartitionedCallPartitionedCall)dense_51/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_123_layer_call_and_return_conditional_losses_5838792 
activation_123/PartitionedCall?
/batch_normalization_106/StatefulPartitionedCallStatefulPartitionedCall'activation_123/PartitionedCall:output:0batch_normalization_106_583913batch_normalization_106_583915batch_normalization_106_583917batch_normalization_106_583919*
Tin	
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_106_layer_call_and_return_conditional_losses_58316621
/batch_normalization_106/StatefulPartitionedCall?
"dropout_52/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_106/StatefulPartitionedCall:output:0#^dropout_51/StatefulPartitionedCall*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_52_layer_call_and_return_conditional_losses_5839342$
"dropout_52/StatefulPartitionedCall?
flatten_35/PartitionedCallPartitionedCall+dropout_52/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_flatten_35_layer_call_and_return_conditional_losses_5839582
flatten_35/PartitionedCall?
 dense_52/StatefulPartitionedCallStatefulPartitionedCall#flatten_35/PartitionedCall:output:0dense_52_583987dense_52_583989*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *M
fHRF
D__inference_dense_52_layer_call_and_return_conditional_losses_5839762"
 dense_52/StatefulPartitionedCall?
activation_124/PartitionedCallPartitionedCall)dense_52/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_124_layer_call_and_return_conditional_losses_5839972 
activation_124/PartitionedCall?
/batch_normalization_107/StatefulPartitionedCallStatefulPartitionedCall'activation_124/PartitionedCall:output:0batch_normalization_107_584031batch_normalization_107_584033batch_normalization_107_584035batch_normalization_107_584037*
Tin	
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_107_layer_call_and_return_conditional_losses_58330621
/batch_normalization_107/StatefulPartitionedCall?
"dropout_53/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_107/StatefulPartitionedCall:output:0#^dropout_52/StatefulPartitionedCall*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_53_layer_call_and_return_conditional_losses_5840522$
"dropout_53/StatefulPartitionedCall?
 dense_53/StatefulPartitionedCallStatefulPartitionedCall+dropout_53/StatefulPartitionedCall:output:0dense_53_584091dense_53_584093*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *M
fHRF
D__inference_dense_53_layer_call_and_return_conditional_losses_5840802"
 dense_53/StatefulPartitionedCall?
activation_125/PartitionedCallPartitionedCall)dense_53/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_125_layer_call_and_return_conditional_losses_5841012 
activation_125/PartitionedCall?
IdentityIdentity'activation_125/PartitionedCall:output:00^batch_normalization_102/StatefulPartitionedCall0^batch_normalization_103/StatefulPartitionedCall0^batch_normalization_104/StatefulPartitionedCall0^batch_normalization_105/StatefulPartitionedCall0^batch_normalization_106/StatefulPartitionedCall0^batch_normalization_107/StatefulPartitionedCall"^conv2d_68/StatefulPartitionedCall"^conv2d_69/StatefulPartitionedCall"^conv2d_70/StatefulPartitionedCall"^conv2d_71/StatefulPartitionedCall!^dense_51/StatefulPartitionedCall!^dense_52/StatefulPartitionedCall!^dense_53/StatefulPartitionedCall#^dropout_51/StatefulPartitionedCall#^dropout_52/StatefulPartitionedCall#^dropout_53/StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::2b
/batch_normalization_102/StatefulPartitionedCall/batch_normalization_102/StatefulPartitionedCall2b
/batch_normalization_103/StatefulPartitionedCall/batch_normalization_103/StatefulPartitionedCall2b
/batch_normalization_104/StatefulPartitionedCall/batch_normalization_104/StatefulPartitionedCall2b
/batch_normalization_105/StatefulPartitionedCall/batch_normalization_105/StatefulPartitionedCall2b
/batch_normalization_106/StatefulPartitionedCall/batch_normalization_106/StatefulPartitionedCall2b
/batch_normalization_107/StatefulPartitionedCall/batch_normalization_107/StatefulPartitionedCall2F
!conv2d_68/StatefulPartitionedCall!conv2d_68/StatefulPartitionedCall2F
!conv2d_69/StatefulPartitionedCall!conv2d_69/StatefulPartitionedCall2F
!conv2d_70/StatefulPartitionedCall!conv2d_70/StatefulPartitionedCall2F
!conv2d_71/StatefulPartitionedCall!conv2d_71/StatefulPartitionedCall2D
 dense_51/StatefulPartitionedCall dense_51/StatefulPartitionedCall2D
 dense_52/StatefulPartitionedCall dense_52/StatefulPartitionedCall2D
 dense_53/StatefulPartitionedCall dense_53/StatefulPartitionedCall2H
"dropout_51/StatefulPartitionedCall"dropout_51/StatefulPartitionedCall2H
"dropout_52/StatefulPartitionedCall"dropout_52/StatefulPartitionedCall2H
"dropout_53/StatefulPartitionedCall"dropout_53/StatefulPartitionedCall:` \
/
_output_shapes
:?????????@@
)
_user_specified_nameconv2d_68_input
?
?
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_582723

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:<*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:<*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*]
_output_shapesK
I:+???????????????????????????<:<:<:<:<:*
epsilon%o?:*
is_training( 2
FusedBatchNormV3?
IdentityIdentityFusedBatchNormV3:y:0 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*A
_output_shapes/
-:+???????????????????????????<2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????<::::2B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:i e
A
_output_shapes/
-:+???????????????????????????<
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_104_layer_call_fn_585602

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 *A
_output_shapes/
-:+???????????????????????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_5829122
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*A
_output_shapes/
-:+???????????????????????????2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????::::22
StatefulPartitionedCallStatefulPartitionedCall:i e
A
_output_shapes/
-:+???????????????????????????
 
_user_specified_nameinputs
?
f
J__inference_activation_119_layer_call_and_return_conditional_losses_583385

inputs
identityV
ReluReluinputs*
T0*/
_output_shapes
:?????????<<<2
Relun
IdentityIdentityRelu:activations:0*
T0*/
_output_shapes
:?????????<<<2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????<<<:W S
/
_output_shapes
:?????????<<<
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_583749

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??AssignNewValue?AssignNewValue_1?FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
exponential_avg_factor%
?#<2
FusedBatchNormV3?
AssignNewValueAssignVariableOp(fusedbatchnormv3_readvariableop_resourceFusedBatchNormV3:batch_mean:0 ^FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*;
_class1
/-loc:@FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02
AssignNewValue?
AssignNewValue_1AssignVariableOp*fusedbatchnormv3_readvariableop_1_resource!FusedBatchNormV3:batch_variance:0"^FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*=
_class3
1/loc:@FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02
AssignNewValue_1?
IdentityIdentityFusedBatchNormV3:y:0^AssignNewValue^AssignNewValue_1 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????::::2 
AssignNewValueAssignNewValue2$
AssignNewValue_1AssignNewValue_12B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
f
J__inference_activation_123_layer_call_and_return_conditional_losses_583879

inputs
identityO
ReluReluinputs*
T0*(
_output_shapes
:??????????2
Relug
IdentityIdentityRelu:activations:0*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
f
J__inference_activation_121_layer_call_and_return_conditional_losses_585546

inputs
identityV
ReluReluinputs*
T0*/
_output_shapes
:?????????2
Relun
IdentityIdentityRelu:activations:0*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_107_layer_call_fn_586134

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_107_layer_call_and_return_conditional_losses_5833392
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*7
_input_shapes&
$:??????????::::22
StatefulPartitionedCallStatefulPartitionedCall:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_102_layer_call_fn_585352

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<<<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_5834122
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*/
_output_shapes
:?????????<<<2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????<<<::::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????<<<
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_106_layer_call_fn_585972

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_106_layer_call_and_return_conditional_losses_5831662
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*7
_input_shapes&
$:??????????::::22
StatefulPartitionedCallStatefulPartitionedCall:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_585589

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*]
_output_shapesK
I:+???????????????????????????:::::*
epsilon%o?:*
is_training( 2
FusedBatchNormV3?
IdentityIdentityFusedBatchNormV3:y:0 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*A
_output_shapes/
-:+???????????????????????????2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????::::2B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:i e
A
_output_shapes/
-:+???????????????????????????
 
_user_specified_nameinputs
?	
?
E__inference_conv2d_71_layer_call_and_return_conditional_losses_585689

inputs"
conv2d_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?Conv2D/ReadVariableOp?
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
:*
dtype02
Conv2D/ReadVariableOp?
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2
Conv2D?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_585496

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:<*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:<*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*]
_output_shapesK
I:+???????????????????????????<:<:<:<:<:*
epsilon%o?:*
is_training( 2
FusedBatchNormV3?
IdentityIdentityFusedBatchNormV3:y:0 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*A
_output_shapes/
-:+???????????????????????????<2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????<::::2B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:i e
A
_output_shapes/
-:+???????????????????????????<
 
_user_specified_nameinputs
?0
?
S__inference_batch_normalization_107_layer_call_and_return_conditional_losses_583306

inputs
assignmovingavg_583281
assignmovingavg_1_583287)
%batchnorm_mul_readvariableop_resource%
!batchnorm_readvariableop_resource
identity??#AssignMovingAvg/AssignSubVariableOp?AssignMovingAvg/ReadVariableOp?%AssignMovingAvg_1/AssignSubVariableOp? AssignMovingAvg_1/ReadVariableOp?batchnorm/ReadVariableOp?batchnorm/mul/ReadVariableOp?
moments/mean/reduction_indicesConst*
_output_shapes
:*
dtype0*
valueB: 2 
moments/mean/reduction_indices?
moments/meanMeaninputs'moments/mean/reduction_indices:output:0*
T0*
_output_shapes
:	?*
	keep_dims(2
moments/mean}
moments/StopGradientStopGradientmoments/mean:output:0*
T0*
_output_shapes
:	?2
moments/StopGradient?
moments/SquaredDifferenceSquaredDifferenceinputsmoments/StopGradient:output:0*
T0*(
_output_shapes
:??????????2
moments/SquaredDifference?
"moments/variance/reduction_indicesConst*
_output_shapes
:*
dtype0*
valueB: 2$
"moments/variance/reduction_indices?
moments/varianceMeanmoments/SquaredDifference:z:0+moments/variance/reduction_indices:output:0*
T0*
_output_shapes
:	?*
	keep_dims(2
moments/variance?
moments/SqueezeSqueezemoments/mean:output:0*
T0*
_output_shapes	
:?*
squeeze_dims
 2
moments/Squeeze?
moments/Squeeze_1Squeezemoments/variance:output:0*
T0*
_output_shapes	
:?*
squeeze_dims
 2
moments/Squeeze_1?
AssignMovingAvg/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*)
_class
loc:@AssignMovingAvg/583281*
_output_shapes
: *
dtype0*
valueB
 *
?#<2
AssignMovingAvg/decay?
AssignMovingAvg/ReadVariableOpReadVariableOpassignmovingavg_583281*
_output_shapes	
:?*
dtype02 
AssignMovingAvg/ReadVariableOp?
AssignMovingAvg/subSub&AssignMovingAvg/ReadVariableOp:value:0moments/Squeeze:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*)
_class
loc:@AssignMovingAvg/583281*
_output_shapes	
:?2
AssignMovingAvg/sub?
AssignMovingAvg/mulMulAssignMovingAvg/sub:z:0AssignMovingAvg/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*)
_class
loc:@AssignMovingAvg/583281*
_output_shapes	
:?2
AssignMovingAvg/mul?
#AssignMovingAvg/AssignSubVariableOpAssignSubVariableOpassignmovingavg_583281AssignMovingAvg/mul:z:0^AssignMovingAvg/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*)
_class
loc:@AssignMovingAvg/583281*
_output_shapes
 *
dtype02%
#AssignMovingAvg/AssignSubVariableOp?
AssignMovingAvg_1/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*+
_class!
loc:@AssignMovingAvg_1/583287*
_output_shapes
: *
dtype0*
valueB
 *
?#<2
AssignMovingAvg_1/decay?
 AssignMovingAvg_1/ReadVariableOpReadVariableOpassignmovingavg_1_583287*
_output_shapes	
:?*
dtype02"
 AssignMovingAvg_1/ReadVariableOp?
AssignMovingAvg_1/subSub(AssignMovingAvg_1/ReadVariableOp:value:0moments/Squeeze_1:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*+
_class!
loc:@AssignMovingAvg_1/583287*
_output_shapes	
:?2
AssignMovingAvg_1/sub?
AssignMovingAvg_1/mulMulAssignMovingAvg_1/sub:z:0 AssignMovingAvg_1/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*+
_class!
loc:@AssignMovingAvg_1/583287*
_output_shapes	
:?2
AssignMovingAvg_1/mul?
%AssignMovingAvg_1/AssignSubVariableOpAssignSubVariableOpassignmovingavg_1_583287AssignMovingAvg_1/mul:z:0!^AssignMovingAvg_1/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*+
_class!
loc:@AssignMovingAvg_1/583287*
_output_shapes
 *
dtype02'
%AssignMovingAvg_1/AssignSubVariableOpg
batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:2
batchnorm/add/y?
batchnorm/addAddV2moments/Squeeze_1:output:0batchnorm/add/y:output:0*
T0*
_output_shapes	
:?2
batchnorm/addd
batchnorm/RsqrtRsqrtbatchnorm/add:z:0*
T0*
_output_shapes	
:?2
batchnorm/Rsqrt?
batchnorm/mul/ReadVariableOpReadVariableOp%batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype02
batchnorm/mul/ReadVariableOp?
batchnorm/mulMulbatchnorm/Rsqrt:y:0$batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?2
batchnorm/mulw
batchnorm/mul_1Mulinputsbatchnorm/mul:z:0*
T0*(
_output_shapes
:??????????2
batchnorm/mul_1|
batchnorm/mul_2Mulmoments/Squeeze:output:0batchnorm/mul:z:0*
T0*
_output_shapes	
:?2
batchnorm/mul_2?
batchnorm/ReadVariableOpReadVariableOp!batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype02
batchnorm/ReadVariableOp?
batchnorm/subSub batchnorm/ReadVariableOp:value:0batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?2
batchnorm/sub?
batchnorm/add_1AddV2batchnorm/mul_1:z:0batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????2
batchnorm/add_1?
IdentityIdentitybatchnorm/add_1:z:0$^AssignMovingAvg/AssignSubVariableOp^AssignMovingAvg/ReadVariableOp&^AssignMovingAvg_1/AssignSubVariableOp!^AssignMovingAvg_1/ReadVariableOp^batchnorm/ReadVariableOp^batchnorm/mul/ReadVariableOp*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*7
_input_shapes&
$:??????????::::2J
#AssignMovingAvg/AssignSubVariableOp#AssignMovingAvg/AssignSubVariableOp2@
AssignMovingAvg/ReadVariableOpAssignMovingAvg/ReadVariableOp2N
%AssignMovingAvg_1/AssignSubVariableOp%AssignMovingAvg_1/AssignSubVariableOp2D
 AssignMovingAvg_1/ReadVariableOp AssignMovingAvg_1/ReadVariableOp24
batchnorm/ReadVariableOpbatchnorm/ReadVariableOp2<
batchnorm/mul/ReadVariableOpbatchnorm/mul/ReadVariableOp:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
b
F__inference_flatten_35_layer_call_and_return_conditional_losses_586018

inputs
identity_
ConstConst*
_output_shapes
:*
dtype0*
valueB"????,  2
Consth
ReshapeReshapeinputsConst:output:0*
T0*(
_output_shapes
:??????????2	
Reshapee
IdentityIdentityReshape:output:0*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
d
+__inference_dropout_51_layer_call_fn_585858

inputs
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_51_layer_call_and_return_conditional_losses_5838162
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
d
F__inference_dropout_51_layer_call_and_return_conditional_losses_585853

inputs

identity_1b
IdentityIdentityinputs*
T0*/
_output_shapes
:?????????2

Identityq

Identity_1IdentityIdentity:output:0*
T0*/
_output_shapes
:?????????2

Identity_1"!

identity_1Identity_1:output:0*.
_input_shapes
:?????????:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
f
J__inference_activation_124_layer_call_and_return_conditional_losses_586047

inputs
identityO
ReluReluinputs*
T0*(
_output_shapes
:??????????2
Relug
IdentityIdentityRelu:activations:0*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_585571

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??AssignNewValue?AssignNewValue_1?FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*]
_output_shapesK
I:+???????????????????????????:::::*
epsilon%o?:*
exponential_avg_factor%
?#<2
FusedBatchNormV3?
AssignNewValueAssignVariableOp(fusedbatchnormv3_readvariableop_resourceFusedBatchNormV3:batch_mean:0 ^FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*;
_class1
/-loc:@FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02
AssignNewValue?
AssignNewValue_1AssignVariableOp*fusedbatchnormv3_readvariableop_1_resource!FusedBatchNormV3:batch_variance:0"^FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*=
_class3
1/loc:@FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02
AssignNewValue_1?
IdentityIdentityFusedBatchNormV3:y:0^AssignNewValue^AssignNewValue_1 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*A
_output_shapes/
-:+???????????????????????????2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????::::2 
AssignNewValueAssignNewValue2$
AssignNewValue_1AssignNewValue_12B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:i e
A
_output_shapes/
-:+???????????????????????????
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_104_layer_call_fn_585666

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_5836372
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????::::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
f
J__inference_activation_122_layer_call_and_return_conditional_losses_583722

inputs
identityV
ReluReluinputs*
T0*/
_output_shapes
:?????????2
Relun
IdentityIdentityRelu:activations:0*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
f
J__inference_activation_122_layer_call_and_return_conditional_losses_585703

inputs
identityV
ReluReluinputs*
T0*/
_output_shapes
:?????????2
Relun
IdentityIdentityRelu:activations:0*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_585432

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:<*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:<*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????88<:<:<:<:<:*
epsilon%o?:*
is_training( 2
FusedBatchNormV3?
IdentityIdentityFusedBatchNormV3:y:0 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*/
_output_shapes
:?????????88<2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????88<::::2B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:W S
/
_output_shapes
:?????????88<
 
_user_specified_nameinputs
?
h
L__inference_max_pooling2d_34_layer_call_and_return_conditional_losses_582844

inputs
identity?
MaxPoolMaxPoolinputs*J
_output_shapes8
6:4????????????????????????????????????*
ksize
*
paddingVALID*
strides
2	
MaxPool?
IdentityIdentityMaxPool:output:0*
T0*J
_output_shapes8
6:4????????????????????????????????????2

Identity"
identityIdentity:output:0*I
_input_shapes8
6:4????????????????????????????????????:r n
J
_output_shapes8
6:4????????????????????????????????????
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_104_layer_call_fn_585679

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_5836552
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????::::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_105_layer_call_fn_585759

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_5837492
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????::::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_585746

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
is_training( 2
FusedBatchNormV3?
IdentityIdentityFusedBatchNormV3:y:0 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????::::2B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
.__inference_sequential_17_layer_call_fn_585208

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
	unknown_3
	unknown_4
	unknown_5
	unknown_6
	unknown_7
	unknown_8
	unknown_9

unknown_10

unknown_11

unknown_12

unknown_13

unknown_14

unknown_15

unknown_16

unknown_17

unknown_18

unknown_19

unknown_20

unknown_21

unknown_22

unknown_23

unknown_24

unknown_25

unknown_26

unknown_27

unknown_28

unknown_29

unknown_30

unknown_31

unknown_32

unknown_33

unknown_34

unknown_35

unknown_36
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
unknown_10
unknown_11
unknown_12
unknown_13
unknown_14
unknown_15
unknown_16
unknown_17
unknown_18
unknown_19
unknown_20
unknown_21
unknown_22
unknown_23
unknown_24
unknown_25
unknown_26
unknown_27
unknown_28
unknown_29
unknown_30
unknown_31
unknown_32
unknown_33
unknown_34
unknown_35
unknown_36*2
Tin+
)2'*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*H
_read_only_resource_inputs*
(&	
 !"#$%&*0
config_proto 

CPU

GPU2*0J 8? *R
fMRK
I__inference_sequential_17_layer_call_and_return_conditional_losses_5845152
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????@@
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_106_layer_call_and_return_conditional_losses_585959

inputs%
!batchnorm_readvariableop_resource)
%batchnorm_mul_readvariableop_resource'
#batchnorm_readvariableop_1_resource'
#batchnorm_readvariableop_2_resource
identity??batchnorm/ReadVariableOp?batchnorm/ReadVariableOp_1?batchnorm/ReadVariableOp_2?batchnorm/mul/ReadVariableOp?
batchnorm/ReadVariableOpReadVariableOp!batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype02
batchnorm/ReadVariableOpg
batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:2
batchnorm/add/y?
batchnorm/addAddV2 batchnorm/ReadVariableOp:value:0batchnorm/add/y:output:0*
T0*
_output_shapes	
:?2
batchnorm/addd
batchnorm/RsqrtRsqrtbatchnorm/add:z:0*
T0*
_output_shapes	
:?2
batchnorm/Rsqrt?
batchnorm/mul/ReadVariableOpReadVariableOp%batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype02
batchnorm/mul/ReadVariableOp?
batchnorm/mulMulbatchnorm/Rsqrt:y:0$batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?2
batchnorm/mulw
batchnorm/mul_1Mulinputsbatchnorm/mul:z:0*
T0*(
_output_shapes
:??????????2
batchnorm/mul_1?
batchnorm/ReadVariableOp_1ReadVariableOp#batchnorm_readvariableop_1_resource*
_output_shapes	
:?*
dtype02
batchnorm/ReadVariableOp_1?
batchnorm/mul_2Mul"batchnorm/ReadVariableOp_1:value:0batchnorm/mul:z:0*
T0*
_output_shapes	
:?2
batchnorm/mul_2?
batchnorm/ReadVariableOp_2ReadVariableOp#batchnorm_readvariableop_2_resource*
_output_shapes	
:?*
dtype02
batchnorm/ReadVariableOp_2?
batchnorm/subSub"batchnorm/ReadVariableOp_2:value:0batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?2
batchnorm/sub?
batchnorm/add_1AddV2batchnorm/mul_1:z:0batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????2
batchnorm/add_1?
IdentityIdentitybatchnorm/add_1:z:0^batchnorm/ReadVariableOp^batchnorm/ReadVariableOp_1^batchnorm/ReadVariableOp_2^batchnorm/mul/ReadVariableOp*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*7
_input_shapes&
$:??????????::::24
batchnorm/ReadVariableOpbatchnorm/ReadVariableOp28
batchnorm/ReadVariableOp_1batchnorm/ReadVariableOp_128
batchnorm/ReadVariableOp_2batchnorm/ReadVariableOp_22<
batchnorm/mul/ReadVariableOpbatchnorm/mul/ReadVariableOp:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
G
+__inference_flatten_34_layer_call_fn_585874

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????!* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_flatten_34_layer_call_and_return_conditional_losses_5838402
PartitionedCallm
IdentityIdentityPartitionedCall:output:0*
T0*(
_output_shapes
:??????????!2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?0
?
S__inference_batch_normalization_106_layer_call_and_return_conditional_losses_585939

inputs
assignmovingavg_585914
assignmovingavg_1_585920)
%batchnorm_mul_readvariableop_resource%
!batchnorm_readvariableop_resource
identity??#AssignMovingAvg/AssignSubVariableOp?AssignMovingAvg/ReadVariableOp?%AssignMovingAvg_1/AssignSubVariableOp? AssignMovingAvg_1/ReadVariableOp?batchnorm/ReadVariableOp?batchnorm/mul/ReadVariableOp?
moments/mean/reduction_indicesConst*
_output_shapes
:*
dtype0*
valueB: 2 
moments/mean/reduction_indices?
moments/meanMeaninputs'moments/mean/reduction_indices:output:0*
T0*
_output_shapes
:	?*
	keep_dims(2
moments/mean}
moments/StopGradientStopGradientmoments/mean:output:0*
T0*
_output_shapes
:	?2
moments/StopGradient?
moments/SquaredDifferenceSquaredDifferenceinputsmoments/StopGradient:output:0*
T0*(
_output_shapes
:??????????2
moments/SquaredDifference?
"moments/variance/reduction_indicesConst*
_output_shapes
:*
dtype0*
valueB: 2$
"moments/variance/reduction_indices?
moments/varianceMeanmoments/SquaredDifference:z:0+moments/variance/reduction_indices:output:0*
T0*
_output_shapes
:	?*
	keep_dims(2
moments/variance?
moments/SqueezeSqueezemoments/mean:output:0*
T0*
_output_shapes	
:?*
squeeze_dims
 2
moments/Squeeze?
moments/Squeeze_1Squeezemoments/variance:output:0*
T0*
_output_shapes	
:?*
squeeze_dims
 2
moments/Squeeze_1?
AssignMovingAvg/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*)
_class
loc:@AssignMovingAvg/585914*
_output_shapes
: *
dtype0*
valueB
 *
?#<2
AssignMovingAvg/decay?
AssignMovingAvg/ReadVariableOpReadVariableOpassignmovingavg_585914*
_output_shapes	
:?*
dtype02 
AssignMovingAvg/ReadVariableOp?
AssignMovingAvg/subSub&AssignMovingAvg/ReadVariableOp:value:0moments/Squeeze:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*)
_class
loc:@AssignMovingAvg/585914*
_output_shapes	
:?2
AssignMovingAvg/sub?
AssignMovingAvg/mulMulAssignMovingAvg/sub:z:0AssignMovingAvg/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*)
_class
loc:@AssignMovingAvg/585914*
_output_shapes	
:?2
AssignMovingAvg/mul?
#AssignMovingAvg/AssignSubVariableOpAssignSubVariableOpassignmovingavg_585914AssignMovingAvg/mul:z:0^AssignMovingAvg/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*)
_class
loc:@AssignMovingAvg/585914*
_output_shapes
 *
dtype02%
#AssignMovingAvg/AssignSubVariableOp?
AssignMovingAvg_1/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*+
_class!
loc:@AssignMovingAvg_1/585920*
_output_shapes
: *
dtype0*
valueB
 *
?#<2
AssignMovingAvg_1/decay?
 AssignMovingAvg_1/ReadVariableOpReadVariableOpassignmovingavg_1_585920*
_output_shapes	
:?*
dtype02"
 AssignMovingAvg_1/ReadVariableOp?
AssignMovingAvg_1/subSub(AssignMovingAvg_1/ReadVariableOp:value:0moments/Squeeze_1:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*+
_class!
loc:@AssignMovingAvg_1/585920*
_output_shapes	
:?2
AssignMovingAvg_1/sub?
AssignMovingAvg_1/mulMulAssignMovingAvg_1/sub:z:0 AssignMovingAvg_1/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*+
_class!
loc:@AssignMovingAvg_1/585920*
_output_shapes	
:?2
AssignMovingAvg_1/mul?
%AssignMovingAvg_1/AssignSubVariableOpAssignSubVariableOpassignmovingavg_1_585920AssignMovingAvg_1/mul:z:0!^AssignMovingAvg_1/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*+
_class!
loc:@AssignMovingAvg_1/585920*
_output_shapes
 *
dtype02'
%AssignMovingAvg_1/AssignSubVariableOpg
batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:2
batchnorm/add/y?
batchnorm/addAddV2moments/Squeeze_1:output:0batchnorm/add/y:output:0*
T0*
_output_shapes	
:?2
batchnorm/addd
batchnorm/RsqrtRsqrtbatchnorm/add:z:0*
T0*
_output_shapes	
:?2
batchnorm/Rsqrt?
batchnorm/mul/ReadVariableOpReadVariableOp%batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype02
batchnorm/mul/ReadVariableOp?
batchnorm/mulMulbatchnorm/Rsqrt:y:0$batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?2
batchnorm/mulw
batchnorm/mul_1Mulinputsbatchnorm/mul:z:0*
T0*(
_output_shapes
:??????????2
batchnorm/mul_1|
batchnorm/mul_2Mulmoments/Squeeze:output:0batchnorm/mul:z:0*
T0*
_output_shapes	
:?2
batchnorm/mul_2?
batchnorm/ReadVariableOpReadVariableOp!batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype02
batchnorm/ReadVariableOp?
batchnorm/subSub batchnorm/ReadVariableOp:value:0batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?2
batchnorm/sub?
batchnorm/add_1AddV2batchnorm/mul_1:z:0batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????2
batchnorm/add_1?
IdentityIdentitybatchnorm/add_1:z:0$^AssignMovingAvg/AssignSubVariableOp^AssignMovingAvg/ReadVariableOp&^AssignMovingAvg_1/AssignSubVariableOp!^AssignMovingAvg_1/ReadVariableOp^batchnorm/ReadVariableOp^batchnorm/mul/ReadVariableOp*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*7
_input_shapes&
$:??????????::::2J
#AssignMovingAvg/AssignSubVariableOp#AssignMovingAvg/AssignSubVariableOp2@
AssignMovingAvg/ReadVariableOpAssignMovingAvg/ReadVariableOp2N
%AssignMovingAvg_1/AssignSubVariableOp%AssignMovingAvg_1/AssignSubVariableOp2D
 AssignMovingAvg_1/ReadVariableOp AssignMovingAvg_1/ReadVariableOp24
batchnorm/ReadVariableOpbatchnorm/ReadVariableOp2<
batchnorm/mul/ReadVariableOpbatchnorm/mul/ReadVariableOp:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
b
F__inference_flatten_34_layer_call_and_return_conditional_losses_583840

inputs
identity_
ConstConst*
_output_shapes
:*
dtype0*
valueB"?????  2
Consth
ReshapeReshapeinputsConst:output:0*
T0*(
_output_shapes
:??????????!2	
Reshapee
IdentityIdentityReshape:output:0*
T0*(
_output_shapes
:??????????!2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
b
F__inference_flatten_35_layer_call_and_return_conditional_losses_583958

inputs
identity_
ConstConst*
_output_shapes
:*
dtype0*
valueB"????,  2
Consth
ReshapeReshapeinputsConst:output:0*
T0*(
_output_shapes
:??????????2	
Reshapee
IdentityIdentityReshape:output:0*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
~
)__inference_dense_52_layer_call_fn_586042

inputs
unknown
	unknown_0
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *M
fHRF
D__inference_dense_52_layer_call_and_return_conditional_losses_5839762
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*/
_input_shapes
:??????????::22
StatefulPartitionedCallStatefulPartitionedCall:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_103_layer_call_fn_585445

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????88<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_5835242
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*/
_output_shapes
:?????????88<2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????88<::::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????88<
 
_user_specified_nameinputs
?
G
+__inference_dropout_53_layer_call_fn_586161

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_53_layer_call_and_return_conditional_losses_5840572
PartitionedCallm
IdentityIdentityPartitionedCall:output:0*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
?
.__inference_sequential_17_layer_call_fn_585127

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
	unknown_3
	unknown_4
	unknown_5
	unknown_6
	unknown_7
	unknown_8
	unknown_9

unknown_10

unknown_11

unknown_12

unknown_13

unknown_14

unknown_15

unknown_16

unknown_17

unknown_18

unknown_19

unknown_20

unknown_21

unknown_22

unknown_23

unknown_24

unknown_25

unknown_26

unknown_27

unknown_28

unknown_29

unknown_30

unknown_31

unknown_32

unknown_33

unknown_34

unknown_35

unknown_36
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
unknown_10
unknown_11
unknown_12
unknown_13
unknown_14
unknown_15
unknown_16
unknown_17
unknown_18
unknown_19
unknown_20
unknown_21
unknown_22
unknown_23
unknown_24
unknown_25
unknown_26
unknown_27
unknown_28
unknown_29
unknown_30
unknown_31
unknown_32
unknown_33
unknown_34
unknown_35
unknown_36*2
Tin+
)2'*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*<
_read_only_resource_inputs
	
 #$%&*0
config_proto 

CPU

GPU2*0J 8? *R
fMRK
I__inference_sequential_17_layer_call_and_return_conditional_losses_5843272
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????@@
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_107_layer_call_fn_586121

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_107_layer_call_and_return_conditional_losses_5833062
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*7
_input_shapes&
$:??????????::::22
StatefulPartitionedCallStatefulPartitionedCall:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_585728

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??AssignNewValue?AssignNewValue_1?FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
exponential_avg_factor%
?#<2
FusedBatchNormV3?
AssignNewValueAssignVariableOp(fusedbatchnormv3_readvariableop_resourceFusedBatchNormV3:batch_mean:0 ^FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*;
_class1
/-loc:@FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02
AssignNewValue?
AssignNewValue_1AssignVariableOp*fusedbatchnormv3_readvariableop_1_resource!FusedBatchNormV3:batch_variance:0"^FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*=
_class3
1/loc:@FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02
AssignNewValue_1?
IdentityIdentityFusedBatchNormV3:y:0^AssignNewValue^AssignNewValue_1 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????::::2 
AssignNewValueAssignNewValue2$
AssignNewValue_1AssignNewValue_12B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_582943

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*]
_output_shapesK
I:+???????????????????????????:::::*
epsilon%o?:*
is_training( 2
FusedBatchNormV3?
IdentityIdentityFusedBatchNormV3:y:0 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*A
_output_shapes/
-:+???????????????????????????2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????::::2B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:i e
A
_output_shapes/
-:+???????????????????????????
 
_user_specified_nameinputs
?

*__inference_conv2d_69_layer_call_fn_585384

inputs
unknown
	unknown_0
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????88<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_69_layer_call_and_return_conditional_losses_5834762
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*/
_output_shapes
:?????????88<2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????<<<::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????<<<
 
_user_specified_nameinputs
?
M
1__inference_max_pooling2d_34_layer_call_fn_582850

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *J
_output_shapes8
6:4????????????????????????????????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *U
fPRN
L__inference_max_pooling2d_34_layer_call_and_return_conditional_losses_5828442
PartitionedCall?
IdentityIdentityPartitionedCall:output:0*
T0*J
_output_shapes8
6:4????????????????????????????????????2

Identity"
identityIdentity:output:0*I
_input_shapes8
6:4????????????????????????????????????:r n
J
_output_shapes8
6:4????????????????????????????????????
 
_user_specified_nameinputs
?
K
/__inference_activation_121_layer_call_fn_585551

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_121_layer_call_and_return_conditional_losses_5836102
PartitionedCallt
IdentityIdentityPartitionedCall:output:0*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_583637

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??AssignNewValue?AssignNewValue_1?FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
exponential_avg_factor%
?#<2
FusedBatchNormV3?
AssignNewValueAssignVariableOp(fusedbatchnormv3_readvariableop_resourceFusedBatchNormV3:batch_mean:0 ^FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*;
_class1
/-loc:@FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02
AssignNewValue?
AssignNewValue_1AssignVariableOp*fusedbatchnormv3_readvariableop_1_resource!FusedBatchNormV3:batch_variance:0"^FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*=
_class3
1/loc:@FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02
AssignNewValue_1?
IdentityIdentityFusedBatchNormV3:y:0^AssignNewValue^AssignNewValue_1 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????::::2 
AssignNewValueAssignNewValue2$
AssignNewValue_1AssignNewValue_12B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_583767

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
is_training( 2
FusedBatchNormV3?
IdentityIdentityFusedBatchNormV3:y:0 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????::::2B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_102_layer_call_fn_585301

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 *A
_output_shapes/
-:+???????????????????????????<*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_5827232
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*A
_output_shapes/
-:+???????????????????????????<2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????<::::22
StatefulPartitionedCallStatefulPartitionedCall:i e
A
_output_shapes/
-:+???????????????????????????<
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_585257

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??AssignNewValue?AssignNewValue_1?FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:<*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:<*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*]
_output_shapesK
I:+???????????????????????????<:<:<:<:<:*
epsilon%o?:*
exponential_avg_factor%
?#<2
FusedBatchNormV3?
AssignNewValueAssignVariableOp(fusedbatchnormv3_readvariableop_resourceFusedBatchNormV3:batch_mean:0 ^FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*;
_class1
/-loc:@FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02
AssignNewValue?
AssignNewValue_1AssignVariableOp*fusedbatchnormv3_readvariableop_1_resource!FusedBatchNormV3:batch_variance:0"^FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*=
_class3
1/loc:@FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02
AssignNewValue_1?
IdentityIdentityFusedBatchNormV3:y:0^AssignNewValue^AssignNewValue_1 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*A
_output_shapes/
-:+???????????????????????????<2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????<::::2 
AssignNewValueAssignNewValue2$
AssignNewValue_1AssignNewValue_12B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:i e
A
_output_shapes/
-:+???????????????????????????<
 
_user_specified_nameinputs
?	
?
D__inference_dense_51_layer_call_and_return_conditional_losses_583858

inputs"
matmul_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?MatMul/ReadVariableOp?
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource* 
_output_shapes
:
?!?*
dtype02
MatMul/ReadVariableOpt
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
MatMul?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes	
:?*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*/
_input_shapes
:??????????!::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:P L
(
_output_shapes
:??????????!
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_585478

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??AssignNewValue?AssignNewValue_1?FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:<*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:<*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*]
_output_shapesK
I:+???????????????????????????<:<:<:<:<:*
epsilon%o?:*
exponential_avg_factor%
?#<2
FusedBatchNormV3?
AssignNewValueAssignVariableOp(fusedbatchnormv3_readvariableop_resourceFusedBatchNormV3:batch_mean:0 ^FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*;
_class1
/-loc:@FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02
AssignNewValue?
AssignNewValue_1AssignVariableOp*fusedbatchnormv3_readvariableop_1_resource!FusedBatchNormV3:batch_variance:0"^FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*=
_class3
1/loc:@FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02
AssignNewValue_1?
IdentityIdentityFusedBatchNormV3:y:0^AssignNewValue^AssignNewValue_1 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*A
_output_shapes/
-:+???????????????????????????<2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????<::::2 
AssignNewValueAssignNewValue2$
AssignNewValue_1AssignNewValue_12B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:i e
A
_output_shapes/
-:+???????????????????????????<
 
_user_specified_nameinputs
?

*__inference_conv2d_68_layer_call_fn_585227

inputs
unknown
	unknown_0
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<<<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_68_layer_call_and_return_conditional_losses_5833642
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*/
_output_shapes
:?????????<<<2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????@@::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????@@
 
_user_specified_nameinputs
?
?
$__inference_signature_wrapper_584685
conv2d_68_input
unknown
	unknown_0
	unknown_1
	unknown_2
	unknown_3
	unknown_4
	unknown_5
	unknown_6
	unknown_7
	unknown_8
	unknown_9

unknown_10

unknown_11

unknown_12

unknown_13

unknown_14

unknown_15

unknown_16

unknown_17

unknown_18

unknown_19

unknown_20

unknown_21

unknown_22

unknown_23

unknown_24

unknown_25

unknown_26

unknown_27

unknown_28

unknown_29

unknown_30

unknown_31

unknown_32

unknown_33

unknown_34

unknown_35

unknown_36
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallconv2d_68_inputunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
unknown_10
unknown_11
unknown_12
unknown_13
unknown_14
unknown_15
unknown_16
unknown_17
unknown_18
unknown_19
unknown_20
unknown_21
unknown_22
unknown_23
unknown_24
unknown_25
unknown_26
unknown_27
unknown_28
unknown_29
unknown_30
unknown_31
unknown_32
unknown_33
unknown_34
unknown_35
unknown_36*2
Tin+
)2'*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*H
_read_only_resource_inputs*
(&	
 !"#$%&*0
config_proto 

CPU

GPU2*0J 8? **
f%R#
!__inference__wrapped_model_5826302
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::22
StatefulPartitionedCallStatefulPartitionedCall:` \
/
_output_shapes
:?????????@@
)
_user_specified_nameconv2d_68_input
?
?
8__inference_batch_normalization_102_layer_call_fn_585365

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<<<*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_5834302
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*/
_output_shapes
:?????????<<<2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????<<<::::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????<<<
 
_user_specified_nameinputs
?
?
.__inference_sequential_17_layer_call_fn_584594
conv2d_68_input
unknown
	unknown_0
	unknown_1
	unknown_2
	unknown_3
	unknown_4
	unknown_5
	unknown_6
	unknown_7
	unknown_8
	unknown_9

unknown_10

unknown_11

unknown_12

unknown_13

unknown_14

unknown_15

unknown_16

unknown_17

unknown_18

unknown_19

unknown_20

unknown_21

unknown_22

unknown_23

unknown_24

unknown_25

unknown_26

unknown_27

unknown_28

unknown_29

unknown_30

unknown_31

unknown_32

unknown_33

unknown_34

unknown_35

unknown_36
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallconv2d_68_inputunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
unknown_10
unknown_11
unknown_12
unknown_13
unknown_14
unknown_15
unknown_16
unknown_17
unknown_18
unknown_19
unknown_20
unknown_21
unknown_22
unknown_23
unknown_24
unknown_25
unknown_26
unknown_27
unknown_28
unknown_29
unknown_30
unknown_31
unknown_32
unknown_33
unknown_34
unknown_35
unknown_36*2
Tin+
)2'*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*H
_read_only_resource_inputs*
(&	
 !"#$%&*0
config_proto 

CPU

GPU2*0J 8? *R
fMRK
I__inference_sequential_17_layer_call_and_return_conditional_losses_5845152
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::22
StatefulPartitionedCallStatefulPartitionedCall:` \
/
_output_shapes
:?????????@@
)
_user_specified_nameconv2d_68_input
?
?
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_585810

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*]
_output_shapesK
I:+???????????????????????????:::::*
epsilon%o?:*
is_training( 2
FusedBatchNormV3?
IdentityIdentityFusedBatchNormV3:y:0 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*A
_output_shapes/
-:+???????????????????????????2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????::::2B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:i e
A
_output_shapes/
-:+???????????????????????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_585339

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:<*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:<*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????<<<:<:<:<:<:*
epsilon%o?:*
is_training( 2
FusedBatchNormV3?
IdentityIdentityFusedBatchNormV3:y:0 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*/
_output_shapes
:?????????<<<2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????<<<::::2B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:W S
/
_output_shapes
:?????????<<<
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_582827

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:<*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:<*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*]
_output_shapesK
I:+???????????????????????????<:<:<:<:<:*
epsilon%o?:*
is_training( 2
FusedBatchNormV3?
IdentityIdentityFusedBatchNormV3:y:0 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*A
_output_shapes/
-:+???????????????????????????<2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????<::::2B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:i e
A
_output_shapes/
-:+???????????????????????????<
 
_user_specified_nameinputs
?	
?
E__inference_conv2d_68_layer_call_and_return_conditional_losses_583364

inputs"
conv2d_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?Conv2D/ReadVariableOp?
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
:<*
dtype02
Conv2D/ReadVariableOp?
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????<<<*
paddingVALID*
strides
2
Conv2D?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:<*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????<<<2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*
T0*/
_output_shapes
:?????????<<<2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????@@::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:?????????@@
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_105_layer_call_fn_585823

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 *A
_output_shapes/
-:+???????????????????????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_5830162
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*A
_output_shapes/
-:+???????????????????????????2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????::::22
StatefulPartitionedCallStatefulPartitionedCall:i e
A
_output_shapes/
-:+???????????????????????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_107_layer_call_and_return_conditional_losses_586108

inputs%
!batchnorm_readvariableop_resource)
%batchnorm_mul_readvariableop_resource'
#batchnorm_readvariableop_1_resource'
#batchnorm_readvariableop_2_resource
identity??batchnorm/ReadVariableOp?batchnorm/ReadVariableOp_1?batchnorm/ReadVariableOp_2?batchnorm/mul/ReadVariableOp?
batchnorm/ReadVariableOpReadVariableOp!batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype02
batchnorm/ReadVariableOpg
batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:2
batchnorm/add/y?
batchnorm/addAddV2 batchnorm/ReadVariableOp:value:0batchnorm/add/y:output:0*
T0*
_output_shapes	
:?2
batchnorm/addd
batchnorm/RsqrtRsqrtbatchnorm/add:z:0*
T0*
_output_shapes	
:?2
batchnorm/Rsqrt?
batchnorm/mul/ReadVariableOpReadVariableOp%batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype02
batchnorm/mul/ReadVariableOp?
batchnorm/mulMulbatchnorm/Rsqrt:y:0$batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?2
batchnorm/mulw
batchnorm/mul_1Mulinputsbatchnorm/mul:z:0*
T0*(
_output_shapes
:??????????2
batchnorm/mul_1?
batchnorm/ReadVariableOp_1ReadVariableOp#batchnorm_readvariableop_1_resource*
_output_shapes	
:?*
dtype02
batchnorm/ReadVariableOp_1?
batchnorm/mul_2Mul"batchnorm/ReadVariableOp_1:value:0batchnorm/mul:z:0*
T0*
_output_shapes	
:?2
batchnorm/mul_2?
batchnorm/ReadVariableOp_2ReadVariableOp#batchnorm_readvariableop_2_resource*
_output_shapes	
:?*
dtype02
batchnorm/ReadVariableOp_2?
batchnorm/subSub"batchnorm/ReadVariableOp_2:value:0batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?2
batchnorm/sub?
batchnorm/add_1AddV2batchnorm/mul_1:z:0batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????2
batchnorm/add_1?
IdentityIdentitybatchnorm/add_1:z:0^batchnorm/ReadVariableOp^batchnorm/ReadVariableOp_1^batchnorm/ReadVariableOp_2^batchnorm/mul/ReadVariableOp*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*7
_input_shapes&
$:??????????::::24
batchnorm/ReadVariableOpbatchnorm/ReadVariableOp28
batchnorm/ReadVariableOp_1batchnorm/ReadVariableOp_128
batchnorm/ReadVariableOp_2batchnorm/ReadVariableOp_22<
batchnorm/mul/ReadVariableOpbatchnorm/mul/ReadVariableOp:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
f
J__inference_activation_121_layer_call_and_return_conditional_losses_583610

inputs
identityV
ReluReluinputs*
T0*/
_output_shapes
:?????????2
Relun
IdentityIdentityRelu:activations:0*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
I__inference_sequential_17_layer_call_and_return_conditional_losses_584515

inputs
conv2d_68_584411
conv2d_68_584413"
batch_normalization_102_584417"
batch_normalization_102_584419"
batch_normalization_102_584421"
batch_normalization_102_584423
conv2d_69_584426
conv2d_69_584428"
batch_normalization_103_584432"
batch_normalization_103_584434"
batch_normalization_103_584436"
batch_normalization_103_584438
conv2d_70_584442
conv2d_70_584444"
batch_normalization_104_584448"
batch_normalization_104_584450"
batch_normalization_104_584452"
batch_normalization_104_584454
conv2d_71_584457
conv2d_71_584459"
batch_normalization_105_584463"
batch_normalization_105_584465"
batch_normalization_105_584467"
batch_normalization_105_584469
dense_51_584475
dense_51_584477"
batch_normalization_106_584481"
batch_normalization_106_584483"
batch_normalization_106_584485"
batch_normalization_106_584487
dense_52_584492
dense_52_584494"
batch_normalization_107_584498"
batch_normalization_107_584500"
batch_normalization_107_584502"
batch_normalization_107_584504
dense_53_584508
dense_53_584510
identity??/batch_normalization_102/StatefulPartitionedCall?/batch_normalization_103/StatefulPartitionedCall?/batch_normalization_104/StatefulPartitionedCall?/batch_normalization_105/StatefulPartitionedCall?/batch_normalization_106/StatefulPartitionedCall?/batch_normalization_107/StatefulPartitionedCall?!conv2d_68/StatefulPartitionedCall?!conv2d_69/StatefulPartitionedCall?!conv2d_70/StatefulPartitionedCall?!conv2d_71/StatefulPartitionedCall? dense_51/StatefulPartitionedCall? dense_52/StatefulPartitionedCall? dense_53/StatefulPartitionedCall?
!conv2d_68/StatefulPartitionedCallStatefulPartitionedCallinputsconv2d_68_584411conv2d_68_584413*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<<<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_68_layer_call_and_return_conditional_losses_5833642#
!conv2d_68/StatefulPartitionedCall?
activation_119/PartitionedCallPartitionedCall*conv2d_68/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<<<* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_119_layer_call_and_return_conditional_losses_5833852 
activation_119/PartitionedCall?
/batch_normalization_102/StatefulPartitionedCallStatefulPartitionedCall'activation_119/PartitionedCall:output:0batch_normalization_102_584417batch_normalization_102_584419batch_normalization_102_584421batch_normalization_102_584423*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<<<*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_58343021
/batch_normalization_102/StatefulPartitionedCall?
!conv2d_69/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_102/StatefulPartitionedCall:output:0conv2d_69_584426conv2d_69_584428*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????88<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_69_layer_call_and_return_conditional_losses_5834762#
!conv2d_69/StatefulPartitionedCall?
activation_120/PartitionedCallPartitionedCall*conv2d_69/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????88<* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_120_layer_call_and_return_conditional_losses_5834972 
activation_120/PartitionedCall?
/batch_normalization_103/StatefulPartitionedCallStatefulPartitionedCall'activation_120/PartitionedCall:output:0batch_normalization_103_584432batch_normalization_103_584434batch_normalization_103_584436batch_normalization_103_584438*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????88<*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_58354221
/batch_normalization_103/StatefulPartitionedCall?
 max_pooling2d_34/PartitionedCallPartitionedCall8batch_normalization_103/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *U
fPRN
L__inference_max_pooling2d_34_layer_call_and_return_conditional_losses_5828442"
 max_pooling2d_34/PartitionedCall?
!conv2d_70/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_34/PartitionedCall:output:0conv2d_70_584442conv2d_70_584444*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_70_layer_call_and_return_conditional_losses_5835892#
!conv2d_70/StatefulPartitionedCall?
activation_121/PartitionedCallPartitionedCall*conv2d_70/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_121_layer_call_and_return_conditional_losses_5836102 
activation_121/PartitionedCall?
/batch_normalization_104/StatefulPartitionedCallStatefulPartitionedCall'activation_121/PartitionedCall:output:0batch_normalization_104_584448batch_normalization_104_584450batch_normalization_104_584452batch_normalization_104_584454*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_58365521
/batch_normalization_104/StatefulPartitionedCall?
!conv2d_71/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_104/StatefulPartitionedCall:output:0conv2d_71_584457conv2d_71_584459*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_71_layer_call_and_return_conditional_losses_5837012#
!conv2d_71/StatefulPartitionedCall?
activation_122/PartitionedCallPartitionedCall*conv2d_71/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_122_layer_call_and_return_conditional_losses_5837222 
activation_122/PartitionedCall?
/batch_normalization_105/StatefulPartitionedCallStatefulPartitionedCall'activation_122/PartitionedCall:output:0batch_normalization_105_584463batch_normalization_105_584465batch_normalization_105_584467batch_normalization_105_584469*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_58376721
/batch_normalization_105/StatefulPartitionedCall?
 max_pooling2d_35/PartitionedCallPartitionedCall8batch_normalization_105/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *U
fPRN
L__inference_max_pooling2d_35_layer_call_and_return_conditional_losses_5830642"
 max_pooling2d_35/PartitionedCall?
dropout_51/PartitionedCallPartitionedCall)max_pooling2d_35/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_51_layer_call_and_return_conditional_losses_5838212
dropout_51/PartitionedCall?
flatten_34/PartitionedCallPartitionedCall#dropout_51/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????!* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_flatten_34_layer_call_and_return_conditional_losses_5838402
flatten_34/PartitionedCall?
 dense_51/StatefulPartitionedCallStatefulPartitionedCall#flatten_34/PartitionedCall:output:0dense_51_584475dense_51_584477*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *M
fHRF
D__inference_dense_51_layer_call_and_return_conditional_losses_5838582"
 dense_51/StatefulPartitionedCall?
activation_123/PartitionedCallPartitionedCall)dense_51/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_123_layer_call_and_return_conditional_losses_5838792 
activation_123/PartitionedCall?
/batch_normalization_106/StatefulPartitionedCallStatefulPartitionedCall'activation_123/PartitionedCall:output:0batch_normalization_106_584481batch_normalization_106_584483batch_normalization_106_584485batch_normalization_106_584487*
Tin	
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_106_layer_call_and_return_conditional_losses_58319921
/batch_normalization_106/StatefulPartitionedCall?
dropout_52/PartitionedCallPartitionedCall8batch_normalization_106/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_52_layer_call_and_return_conditional_losses_5839392
dropout_52/PartitionedCall?
flatten_35/PartitionedCallPartitionedCall#dropout_52/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_flatten_35_layer_call_and_return_conditional_losses_5839582
flatten_35/PartitionedCall?
 dense_52/StatefulPartitionedCallStatefulPartitionedCall#flatten_35/PartitionedCall:output:0dense_52_584492dense_52_584494*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *M
fHRF
D__inference_dense_52_layer_call_and_return_conditional_losses_5839762"
 dense_52/StatefulPartitionedCall?
activation_124/PartitionedCallPartitionedCall)dense_52/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_124_layer_call_and_return_conditional_losses_5839972 
activation_124/PartitionedCall?
/batch_normalization_107/StatefulPartitionedCallStatefulPartitionedCall'activation_124/PartitionedCall:output:0batch_normalization_107_584498batch_normalization_107_584500batch_normalization_107_584502batch_normalization_107_584504*
Tin	
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_107_layer_call_and_return_conditional_losses_58333921
/batch_normalization_107/StatefulPartitionedCall?
dropout_53/PartitionedCallPartitionedCall8batch_normalization_107/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_53_layer_call_and_return_conditional_losses_5840572
dropout_53/PartitionedCall?
 dense_53/StatefulPartitionedCallStatefulPartitionedCall#dropout_53/PartitionedCall:output:0dense_53_584508dense_53_584510*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *M
fHRF
D__inference_dense_53_layer_call_and_return_conditional_losses_5840802"
 dense_53/StatefulPartitionedCall?
activation_125/PartitionedCallPartitionedCall)dense_53/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_125_layer_call_and_return_conditional_losses_5841012 
activation_125/PartitionedCall?
IdentityIdentity'activation_125/PartitionedCall:output:00^batch_normalization_102/StatefulPartitionedCall0^batch_normalization_103/StatefulPartitionedCall0^batch_normalization_104/StatefulPartitionedCall0^batch_normalization_105/StatefulPartitionedCall0^batch_normalization_106/StatefulPartitionedCall0^batch_normalization_107/StatefulPartitionedCall"^conv2d_68/StatefulPartitionedCall"^conv2d_69/StatefulPartitionedCall"^conv2d_70/StatefulPartitionedCall"^conv2d_71/StatefulPartitionedCall!^dense_51/StatefulPartitionedCall!^dense_52/StatefulPartitionedCall!^dense_53/StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::2b
/batch_normalization_102/StatefulPartitionedCall/batch_normalization_102/StatefulPartitionedCall2b
/batch_normalization_103/StatefulPartitionedCall/batch_normalization_103/StatefulPartitionedCall2b
/batch_normalization_104/StatefulPartitionedCall/batch_normalization_104/StatefulPartitionedCall2b
/batch_normalization_105/StatefulPartitionedCall/batch_normalization_105/StatefulPartitionedCall2b
/batch_normalization_106/StatefulPartitionedCall/batch_normalization_106/StatefulPartitionedCall2b
/batch_normalization_107/StatefulPartitionedCall/batch_normalization_107/StatefulPartitionedCall2F
!conv2d_68/StatefulPartitionedCall!conv2d_68/StatefulPartitionedCall2F
!conv2d_69/StatefulPartitionedCall!conv2d_69/StatefulPartitionedCall2F
!conv2d_70/StatefulPartitionedCall!conv2d_70/StatefulPartitionedCall2F
!conv2d_71/StatefulPartitionedCall!conv2d_71/StatefulPartitionedCall2D
 dense_51/StatefulPartitionedCall dense_51/StatefulPartitionedCall2D
 dense_52/StatefulPartitionedCall dense_52/StatefulPartitionedCall2D
 dense_53/StatefulPartitionedCall dense_53/StatefulPartitionedCall:W S
/
_output_shapes
:?????????@@
 
_user_specified_nameinputs
?

*__inference_conv2d_71_layer_call_fn_585698

inputs
unknown
	unknown_0
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_71_layer_call_and_return_conditional_losses_5837012
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????::22
StatefulPartitionedCallStatefulPartitionedCall:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
K
/__inference_activation_122_layer_call_fn_585708

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_122_layer_call_and_return_conditional_losses_5837222
PartitionedCallt
IdentityIdentityPartitionedCall:output:0*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
b
F__inference_flatten_34_layer_call_and_return_conditional_losses_585869

inputs
identity_
ConstConst*
_output_shapes
:*
dtype0*
valueB"?????  2
Consth
ReshapeReshapeinputsConst:output:0*
T0*(
_output_shapes
:??????????!2	
Reshapee
IdentityIdentityReshape:output:0*
T0*(
_output_shapes
:??????????!2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
K
/__inference_activation_124_layer_call_fn_586052

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_124_layer_call_and_return_conditional_losses_5839972
PartitionedCallm
IdentityIdentityPartitionedCall:output:0*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_583524

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??AssignNewValue?AssignNewValue_1?FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:<*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:<*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????88<:<:<:<:<:*
epsilon%o?:*
exponential_avg_factor%
?#<2
FusedBatchNormV3?
AssignNewValueAssignVariableOp(fusedbatchnormv3_readvariableop_resourceFusedBatchNormV3:batch_mean:0 ^FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*;
_class1
/-loc:@FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02
AssignNewValue?
AssignNewValue_1AssignVariableOp*fusedbatchnormv3_readvariableop_1_resource!FusedBatchNormV3:batch_variance:0"^FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*=
_class3
1/loc:@FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02
AssignNewValue_1?
IdentityIdentityFusedBatchNormV3:y:0^AssignNewValue^AssignNewValue_1 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*/
_output_shapes
:?????????88<2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????88<::::2 
AssignNewValueAssignNewValue2$
AssignNewValue_1AssignNewValue_12B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:W S
/
_output_shapes
:?????????88<
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_582796

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??AssignNewValue?AssignNewValue_1?FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:<*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:<*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*]
_output_shapesK
I:+???????????????????????????<:<:<:<:<:*
epsilon%o?:*
exponential_avg_factor%
?#<2
FusedBatchNormV3?
AssignNewValueAssignVariableOp(fusedbatchnormv3_readvariableop_resourceFusedBatchNormV3:batch_mean:0 ^FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*;
_class1
/-loc:@FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02
AssignNewValue?
AssignNewValue_1AssignVariableOp*fusedbatchnormv3_readvariableop_1_resource!FusedBatchNormV3:batch_variance:0"^FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*=
_class3
1/loc:@FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02
AssignNewValue_1?
IdentityIdentityFusedBatchNormV3:y:0^AssignNewValue^AssignNewValue_1 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*A
_output_shapes/
-:+???????????????????????????<2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????<::::2 
AssignNewValueAssignNewValue2$
AssignNewValue_1AssignNewValue_12B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:i e
A
_output_shapes/
-:+???????????????????????????<
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_585321

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??AssignNewValue?AssignNewValue_1?FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:<*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:<*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????<<<:<:<:<:<:*
epsilon%o?:*
exponential_avg_factor%
?#<2
FusedBatchNormV3?
AssignNewValueAssignVariableOp(fusedbatchnormv3_readvariableop_resourceFusedBatchNormV3:batch_mean:0 ^FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*;
_class1
/-loc:@FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02
AssignNewValue?
AssignNewValue_1AssignVariableOp*fusedbatchnormv3_readvariableop_1_resource!FusedBatchNormV3:batch_variance:0"^FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*=
_class3
1/loc:@FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02
AssignNewValue_1?
IdentityIdentityFusedBatchNormV3:y:0^AssignNewValue^AssignNewValue_1 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*/
_output_shapes
:?????????<<<2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????<<<::::2 
AssignNewValueAssignNewValue2$
AssignNewValue_1AssignNewValue_12B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:W S
/
_output_shapes
:?????????<<<
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_583655

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
is_training( 2
FusedBatchNormV3?
IdentityIdentityFusedBatchNormV3:y:0 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*>
_input_shapes-
+:?????????::::2B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
~
)__inference_dense_53_layer_call_fn_586180

inputs
unknown
	unknown_0
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *M
fHRF
D__inference_dense_53_layer_call_and_return_conditional_losses_5840802
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*/
_input_shapes
:??????????::22
StatefulPartitionedCallStatefulPartitionedCall:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?	
?
E__inference_conv2d_70_layer_call_and_return_conditional_losses_585532

inputs"
conv2d_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?Conv2D/ReadVariableOp?
Conv2D/ReadVariableOpReadVariableOpconv2d_readvariableop_resource*&
_output_shapes
:<*
dtype02
Conv2D/ReadVariableOp?
Conv2DConv2DinputsConv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2
Conv2D?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddConv2D:output:0BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^Conv2D/ReadVariableOp*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*6
_input_shapes%
#:?????????<::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
Conv2D/ReadVariableOpConv2D/ReadVariableOp:W S
/
_output_shapes
:?????????<
 
_user_specified_nameinputs
Ƨ
?9
"__inference__traced_restore_586817
file_prefix%
!assignvariableop_conv2d_68_kernel%
!assignvariableop_1_conv2d_68_bias4
0assignvariableop_2_batch_normalization_102_gamma3
/assignvariableop_3_batch_normalization_102_beta:
6assignvariableop_4_batch_normalization_102_moving_mean>
:assignvariableop_5_batch_normalization_102_moving_variance'
#assignvariableop_6_conv2d_69_kernel%
!assignvariableop_7_conv2d_69_bias4
0assignvariableop_8_batch_normalization_103_gamma3
/assignvariableop_9_batch_normalization_103_beta;
7assignvariableop_10_batch_normalization_103_moving_mean?
;assignvariableop_11_batch_normalization_103_moving_variance(
$assignvariableop_12_conv2d_70_kernel&
"assignvariableop_13_conv2d_70_bias5
1assignvariableop_14_batch_normalization_104_gamma4
0assignvariableop_15_batch_normalization_104_beta;
7assignvariableop_16_batch_normalization_104_moving_mean?
;assignvariableop_17_batch_normalization_104_moving_variance(
$assignvariableop_18_conv2d_71_kernel&
"assignvariableop_19_conv2d_71_bias5
1assignvariableop_20_batch_normalization_105_gamma4
0assignvariableop_21_batch_normalization_105_beta;
7assignvariableop_22_batch_normalization_105_moving_mean?
;assignvariableop_23_batch_normalization_105_moving_variance'
#assignvariableop_24_dense_51_kernel%
!assignvariableop_25_dense_51_bias5
1assignvariableop_26_batch_normalization_106_gamma4
0assignvariableop_27_batch_normalization_106_beta;
7assignvariableop_28_batch_normalization_106_moving_mean?
;assignvariableop_29_batch_normalization_106_moving_variance'
#assignvariableop_30_dense_52_kernel%
!assignvariableop_31_dense_52_bias5
1assignvariableop_32_batch_normalization_107_gamma4
0assignvariableop_33_batch_normalization_107_beta;
7assignvariableop_34_batch_normalization_107_moving_mean?
;assignvariableop_35_batch_normalization_107_moving_variance'
#assignvariableop_36_dense_53_kernel%
!assignvariableop_37_dense_53_bias!
assignvariableop_38_adam_iter#
assignvariableop_39_adam_beta_1#
assignvariableop_40_adam_beta_2"
assignvariableop_41_adam_decay*
&assignvariableop_42_adam_learning_rate
assignvariableop_43_total
assignvariableop_44_count
assignvariableop_45_total_1
assignvariableop_46_count_1/
+assignvariableop_47_adam_conv2d_68_kernel_m-
)assignvariableop_48_adam_conv2d_68_bias_m<
8assignvariableop_49_adam_batch_normalization_102_gamma_m;
7assignvariableop_50_adam_batch_normalization_102_beta_m/
+assignvariableop_51_adam_conv2d_69_kernel_m-
)assignvariableop_52_adam_conv2d_69_bias_m<
8assignvariableop_53_adam_batch_normalization_103_gamma_m;
7assignvariableop_54_adam_batch_normalization_103_beta_m/
+assignvariableop_55_adam_conv2d_70_kernel_m-
)assignvariableop_56_adam_conv2d_70_bias_m<
8assignvariableop_57_adam_batch_normalization_104_gamma_m;
7assignvariableop_58_adam_batch_normalization_104_beta_m/
+assignvariableop_59_adam_conv2d_71_kernel_m-
)assignvariableop_60_adam_conv2d_71_bias_m<
8assignvariableop_61_adam_batch_normalization_105_gamma_m;
7assignvariableop_62_adam_batch_normalization_105_beta_m.
*assignvariableop_63_adam_dense_51_kernel_m,
(assignvariableop_64_adam_dense_51_bias_m<
8assignvariableop_65_adam_batch_normalization_106_gamma_m;
7assignvariableop_66_adam_batch_normalization_106_beta_m.
*assignvariableop_67_adam_dense_52_kernel_m,
(assignvariableop_68_adam_dense_52_bias_m<
8assignvariableop_69_adam_batch_normalization_107_gamma_m;
7assignvariableop_70_adam_batch_normalization_107_beta_m.
*assignvariableop_71_adam_dense_53_kernel_m,
(assignvariableop_72_adam_dense_53_bias_m/
+assignvariableop_73_adam_conv2d_68_kernel_v-
)assignvariableop_74_adam_conv2d_68_bias_v<
8assignvariableop_75_adam_batch_normalization_102_gamma_v;
7assignvariableop_76_adam_batch_normalization_102_beta_v/
+assignvariableop_77_adam_conv2d_69_kernel_v-
)assignvariableop_78_adam_conv2d_69_bias_v<
8assignvariableop_79_adam_batch_normalization_103_gamma_v;
7assignvariableop_80_adam_batch_normalization_103_beta_v/
+assignvariableop_81_adam_conv2d_70_kernel_v-
)assignvariableop_82_adam_conv2d_70_bias_v<
8assignvariableop_83_adam_batch_normalization_104_gamma_v;
7assignvariableop_84_adam_batch_normalization_104_beta_v/
+assignvariableop_85_adam_conv2d_71_kernel_v-
)assignvariableop_86_adam_conv2d_71_bias_v<
8assignvariableop_87_adam_batch_normalization_105_gamma_v;
7assignvariableop_88_adam_batch_normalization_105_beta_v.
*assignvariableop_89_adam_dense_51_kernel_v,
(assignvariableop_90_adam_dense_51_bias_v<
8assignvariableop_91_adam_batch_normalization_106_gamma_v;
7assignvariableop_92_adam_batch_normalization_106_beta_v.
*assignvariableop_93_adam_dense_52_kernel_v,
(assignvariableop_94_adam_dense_52_bias_v<
8assignvariableop_95_adam_batch_normalization_107_gamma_v;
7assignvariableop_96_adam_batch_normalization_107_beta_v.
*assignvariableop_97_adam_dense_53_kernel_v,
(assignvariableop_98_adam_dense_53_bias_v
identity_100??AssignVariableOp?AssignVariableOp_1?AssignVariableOp_10?AssignVariableOp_11?AssignVariableOp_12?AssignVariableOp_13?AssignVariableOp_14?AssignVariableOp_15?AssignVariableOp_16?AssignVariableOp_17?AssignVariableOp_18?AssignVariableOp_19?AssignVariableOp_2?AssignVariableOp_20?AssignVariableOp_21?AssignVariableOp_22?AssignVariableOp_23?AssignVariableOp_24?AssignVariableOp_25?AssignVariableOp_26?AssignVariableOp_27?AssignVariableOp_28?AssignVariableOp_29?AssignVariableOp_3?AssignVariableOp_30?AssignVariableOp_31?AssignVariableOp_32?AssignVariableOp_33?AssignVariableOp_34?AssignVariableOp_35?AssignVariableOp_36?AssignVariableOp_37?AssignVariableOp_38?AssignVariableOp_39?AssignVariableOp_4?AssignVariableOp_40?AssignVariableOp_41?AssignVariableOp_42?AssignVariableOp_43?AssignVariableOp_44?AssignVariableOp_45?AssignVariableOp_46?AssignVariableOp_47?AssignVariableOp_48?AssignVariableOp_49?AssignVariableOp_5?AssignVariableOp_50?AssignVariableOp_51?AssignVariableOp_52?AssignVariableOp_53?AssignVariableOp_54?AssignVariableOp_55?AssignVariableOp_56?AssignVariableOp_57?AssignVariableOp_58?AssignVariableOp_59?AssignVariableOp_6?AssignVariableOp_60?AssignVariableOp_61?AssignVariableOp_62?AssignVariableOp_63?AssignVariableOp_64?AssignVariableOp_65?AssignVariableOp_66?AssignVariableOp_67?AssignVariableOp_68?AssignVariableOp_69?AssignVariableOp_7?AssignVariableOp_70?AssignVariableOp_71?AssignVariableOp_72?AssignVariableOp_73?AssignVariableOp_74?AssignVariableOp_75?AssignVariableOp_76?AssignVariableOp_77?AssignVariableOp_78?AssignVariableOp_79?AssignVariableOp_8?AssignVariableOp_80?AssignVariableOp_81?AssignVariableOp_82?AssignVariableOp_83?AssignVariableOp_84?AssignVariableOp_85?AssignVariableOp_86?AssignVariableOp_87?AssignVariableOp_88?AssignVariableOp_89?AssignVariableOp_9?AssignVariableOp_90?AssignVariableOp_91?AssignVariableOp_92?AssignVariableOp_93?AssignVariableOp_94?AssignVariableOp_95?AssignVariableOp_96?AssignVariableOp_97?AssignVariableOp_98?7
RestoreV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:d*
dtype0*?6
value?6B?6dB6layer_with_weights-0/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-0/bias/.ATTRIBUTES/VARIABLE_VALUEB5layer_with_weights-1/gamma/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-1/beta/.ATTRIBUTES/VARIABLE_VALUEB;layer_with_weights-1/moving_mean/.ATTRIBUTES/VARIABLE_VALUEB?layer_with_weights-1/moving_variance/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-2/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-2/bias/.ATTRIBUTES/VARIABLE_VALUEB5layer_with_weights-3/gamma/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-3/beta/.ATTRIBUTES/VARIABLE_VALUEB;layer_with_weights-3/moving_mean/.ATTRIBUTES/VARIABLE_VALUEB?layer_with_weights-3/moving_variance/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-4/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-4/bias/.ATTRIBUTES/VARIABLE_VALUEB5layer_with_weights-5/gamma/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-5/beta/.ATTRIBUTES/VARIABLE_VALUEB;layer_with_weights-5/moving_mean/.ATTRIBUTES/VARIABLE_VALUEB?layer_with_weights-5/moving_variance/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-6/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-6/bias/.ATTRIBUTES/VARIABLE_VALUEB5layer_with_weights-7/gamma/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-7/beta/.ATTRIBUTES/VARIABLE_VALUEB;layer_with_weights-7/moving_mean/.ATTRIBUTES/VARIABLE_VALUEB?layer_with_weights-7/moving_variance/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-8/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-8/bias/.ATTRIBUTES/VARIABLE_VALUEB5layer_with_weights-9/gamma/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-9/beta/.ATTRIBUTES/VARIABLE_VALUEB;layer_with_weights-9/moving_mean/.ATTRIBUTES/VARIABLE_VALUEB?layer_with_weights-9/moving_variance/.ATTRIBUTES/VARIABLE_VALUEB7layer_with_weights-10/kernel/.ATTRIBUTES/VARIABLE_VALUEB5layer_with_weights-10/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-11/gamma/.ATTRIBUTES/VARIABLE_VALUEB5layer_with_weights-11/beta/.ATTRIBUTES/VARIABLE_VALUEB<layer_with_weights-11/moving_mean/.ATTRIBUTES/VARIABLE_VALUEB@layer_with_weights-11/moving_variance/.ATTRIBUTES/VARIABLE_VALUEB7layer_with_weights-12/kernel/.ATTRIBUTES/VARIABLE_VALUEB5layer_with_weights-12/bias/.ATTRIBUTES/VARIABLE_VALUEB)optimizer/iter/.ATTRIBUTES/VARIABLE_VALUEB+optimizer/beta_1/.ATTRIBUTES/VARIABLE_VALUEB+optimizer/beta_2/.ATTRIBUTES/VARIABLE_VALUEB*optimizer/decay/.ATTRIBUTES/VARIABLE_VALUEB2optimizer/learning_rate/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/count/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/1/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/1/count/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-1/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-1/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-3/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-3/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-4/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-4/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-5/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-5/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-6/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-6/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-7/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-7/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-8/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-8/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-9/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-9/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBSlayer_with_weights-10/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-10/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-11/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-11/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBSlayer_with_weights-12/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-12/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-1/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-1/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-3/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-3/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-4/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-4/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-5/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-5/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-6/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-6/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-7/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-7/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-8/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-8/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-9/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-9/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBSlayer_with_weights-10/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-10/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-11/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-11/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBSlayer_with_weights-12/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBQlayer_with_weights-12/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH2
RestoreV2/tensor_names?
RestoreV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:d*
dtype0*?
value?B?dB B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B 2
RestoreV2/shape_and_slices?
	RestoreV2	RestoreV2file_prefixRestoreV2/tensor_names:output:0#RestoreV2/shape_and_slices:output:0"/device:CPU:0*?
_output_shapes?
?::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*r
dtypesh
f2d	2
	RestoreV2g
IdentityIdentityRestoreV2:tensors:0"/device:CPU:0*
T0*
_output_shapes
:2

Identity?
AssignVariableOpAssignVariableOp!assignvariableop_conv2d_68_kernelIdentity:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOpk

Identity_1IdentityRestoreV2:tensors:1"/device:CPU:0*
T0*
_output_shapes
:2

Identity_1?
AssignVariableOp_1AssignVariableOp!assignvariableop_1_conv2d_68_biasIdentity_1:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_1k

Identity_2IdentityRestoreV2:tensors:2"/device:CPU:0*
T0*
_output_shapes
:2

Identity_2?
AssignVariableOp_2AssignVariableOp0assignvariableop_2_batch_normalization_102_gammaIdentity_2:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_2k

Identity_3IdentityRestoreV2:tensors:3"/device:CPU:0*
T0*
_output_shapes
:2

Identity_3?
AssignVariableOp_3AssignVariableOp/assignvariableop_3_batch_normalization_102_betaIdentity_3:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_3k

Identity_4IdentityRestoreV2:tensors:4"/device:CPU:0*
T0*
_output_shapes
:2

Identity_4?
AssignVariableOp_4AssignVariableOp6assignvariableop_4_batch_normalization_102_moving_meanIdentity_4:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_4k

Identity_5IdentityRestoreV2:tensors:5"/device:CPU:0*
T0*
_output_shapes
:2

Identity_5?
AssignVariableOp_5AssignVariableOp:assignvariableop_5_batch_normalization_102_moving_varianceIdentity_5:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_5k

Identity_6IdentityRestoreV2:tensors:6"/device:CPU:0*
T0*
_output_shapes
:2

Identity_6?
AssignVariableOp_6AssignVariableOp#assignvariableop_6_conv2d_69_kernelIdentity_6:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_6k

Identity_7IdentityRestoreV2:tensors:7"/device:CPU:0*
T0*
_output_shapes
:2

Identity_7?
AssignVariableOp_7AssignVariableOp!assignvariableop_7_conv2d_69_biasIdentity_7:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_7k

Identity_8IdentityRestoreV2:tensors:8"/device:CPU:0*
T0*
_output_shapes
:2

Identity_8?
AssignVariableOp_8AssignVariableOp0assignvariableop_8_batch_normalization_103_gammaIdentity_8:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_8k

Identity_9IdentityRestoreV2:tensors:9"/device:CPU:0*
T0*
_output_shapes
:2

Identity_9?
AssignVariableOp_9AssignVariableOp/assignvariableop_9_batch_normalization_103_betaIdentity_9:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_9n
Identity_10IdentityRestoreV2:tensors:10"/device:CPU:0*
T0*
_output_shapes
:2
Identity_10?
AssignVariableOp_10AssignVariableOp7assignvariableop_10_batch_normalization_103_moving_meanIdentity_10:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_10n
Identity_11IdentityRestoreV2:tensors:11"/device:CPU:0*
T0*
_output_shapes
:2
Identity_11?
AssignVariableOp_11AssignVariableOp;assignvariableop_11_batch_normalization_103_moving_varianceIdentity_11:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_11n
Identity_12IdentityRestoreV2:tensors:12"/device:CPU:0*
T0*
_output_shapes
:2
Identity_12?
AssignVariableOp_12AssignVariableOp$assignvariableop_12_conv2d_70_kernelIdentity_12:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_12n
Identity_13IdentityRestoreV2:tensors:13"/device:CPU:0*
T0*
_output_shapes
:2
Identity_13?
AssignVariableOp_13AssignVariableOp"assignvariableop_13_conv2d_70_biasIdentity_13:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_13n
Identity_14IdentityRestoreV2:tensors:14"/device:CPU:0*
T0*
_output_shapes
:2
Identity_14?
AssignVariableOp_14AssignVariableOp1assignvariableop_14_batch_normalization_104_gammaIdentity_14:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_14n
Identity_15IdentityRestoreV2:tensors:15"/device:CPU:0*
T0*
_output_shapes
:2
Identity_15?
AssignVariableOp_15AssignVariableOp0assignvariableop_15_batch_normalization_104_betaIdentity_15:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_15n
Identity_16IdentityRestoreV2:tensors:16"/device:CPU:0*
T0*
_output_shapes
:2
Identity_16?
AssignVariableOp_16AssignVariableOp7assignvariableop_16_batch_normalization_104_moving_meanIdentity_16:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_16n
Identity_17IdentityRestoreV2:tensors:17"/device:CPU:0*
T0*
_output_shapes
:2
Identity_17?
AssignVariableOp_17AssignVariableOp;assignvariableop_17_batch_normalization_104_moving_varianceIdentity_17:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_17n
Identity_18IdentityRestoreV2:tensors:18"/device:CPU:0*
T0*
_output_shapes
:2
Identity_18?
AssignVariableOp_18AssignVariableOp$assignvariableop_18_conv2d_71_kernelIdentity_18:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_18n
Identity_19IdentityRestoreV2:tensors:19"/device:CPU:0*
T0*
_output_shapes
:2
Identity_19?
AssignVariableOp_19AssignVariableOp"assignvariableop_19_conv2d_71_biasIdentity_19:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_19n
Identity_20IdentityRestoreV2:tensors:20"/device:CPU:0*
T0*
_output_shapes
:2
Identity_20?
AssignVariableOp_20AssignVariableOp1assignvariableop_20_batch_normalization_105_gammaIdentity_20:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_20n
Identity_21IdentityRestoreV2:tensors:21"/device:CPU:0*
T0*
_output_shapes
:2
Identity_21?
AssignVariableOp_21AssignVariableOp0assignvariableop_21_batch_normalization_105_betaIdentity_21:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_21n
Identity_22IdentityRestoreV2:tensors:22"/device:CPU:0*
T0*
_output_shapes
:2
Identity_22?
AssignVariableOp_22AssignVariableOp7assignvariableop_22_batch_normalization_105_moving_meanIdentity_22:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_22n
Identity_23IdentityRestoreV2:tensors:23"/device:CPU:0*
T0*
_output_shapes
:2
Identity_23?
AssignVariableOp_23AssignVariableOp;assignvariableop_23_batch_normalization_105_moving_varianceIdentity_23:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_23n
Identity_24IdentityRestoreV2:tensors:24"/device:CPU:0*
T0*
_output_shapes
:2
Identity_24?
AssignVariableOp_24AssignVariableOp#assignvariableop_24_dense_51_kernelIdentity_24:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_24n
Identity_25IdentityRestoreV2:tensors:25"/device:CPU:0*
T0*
_output_shapes
:2
Identity_25?
AssignVariableOp_25AssignVariableOp!assignvariableop_25_dense_51_biasIdentity_25:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_25n
Identity_26IdentityRestoreV2:tensors:26"/device:CPU:0*
T0*
_output_shapes
:2
Identity_26?
AssignVariableOp_26AssignVariableOp1assignvariableop_26_batch_normalization_106_gammaIdentity_26:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_26n
Identity_27IdentityRestoreV2:tensors:27"/device:CPU:0*
T0*
_output_shapes
:2
Identity_27?
AssignVariableOp_27AssignVariableOp0assignvariableop_27_batch_normalization_106_betaIdentity_27:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_27n
Identity_28IdentityRestoreV2:tensors:28"/device:CPU:0*
T0*
_output_shapes
:2
Identity_28?
AssignVariableOp_28AssignVariableOp7assignvariableop_28_batch_normalization_106_moving_meanIdentity_28:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_28n
Identity_29IdentityRestoreV2:tensors:29"/device:CPU:0*
T0*
_output_shapes
:2
Identity_29?
AssignVariableOp_29AssignVariableOp;assignvariableop_29_batch_normalization_106_moving_varianceIdentity_29:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_29n
Identity_30IdentityRestoreV2:tensors:30"/device:CPU:0*
T0*
_output_shapes
:2
Identity_30?
AssignVariableOp_30AssignVariableOp#assignvariableop_30_dense_52_kernelIdentity_30:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_30n
Identity_31IdentityRestoreV2:tensors:31"/device:CPU:0*
T0*
_output_shapes
:2
Identity_31?
AssignVariableOp_31AssignVariableOp!assignvariableop_31_dense_52_biasIdentity_31:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_31n
Identity_32IdentityRestoreV2:tensors:32"/device:CPU:0*
T0*
_output_shapes
:2
Identity_32?
AssignVariableOp_32AssignVariableOp1assignvariableop_32_batch_normalization_107_gammaIdentity_32:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_32n
Identity_33IdentityRestoreV2:tensors:33"/device:CPU:0*
T0*
_output_shapes
:2
Identity_33?
AssignVariableOp_33AssignVariableOp0assignvariableop_33_batch_normalization_107_betaIdentity_33:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_33n
Identity_34IdentityRestoreV2:tensors:34"/device:CPU:0*
T0*
_output_shapes
:2
Identity_34?
AssignVariableOp_34AssignVariableOp7assignvariableop_34_batch_normalization_107_moving_meanIdentity_34:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_34n
Identity_35IdentityRestoreV2:tensors:35"/device:CPU:0*
T0*
_output_shapes
:2
Identity_35?
AssignVariableOp_35AssignVariableOp;assignvariableop_35_batch_normalization_107_moving_varianceIdentity_35:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_35n
Identity_36IdentityRestoreV2:tensors:36"/device:CPU:0*
T0*
_output_shapes
:2
Identity_36?
AssignVariableOp_36AssignVariableOp#assignvariableop_36_dense_53_kernelIdentity_36:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_36n
Identity_37IdentityRestoreV2:tensors:37"/device:CPU:0*
T0*
_output_shapes
:2
Identity_37?
AssignVariableOp_37AssignVariableOp!assignvariableop_37_dense_53_biasIdentity_37:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_37n
Identity_38IdentityRestoreV2:tensors:38"/device:CPU:0*
T0	*
_output_shapes
:2
Identity_38?
AssignVariableOp_38AssignVariableOpassignvariableop_38_adam_iterIdentity_38:output:0"/device:CPU:0*
_output_shapes
 *
dtype0	2
AssignVariableOp_38n
Identity_39IdentityRestoreV2:tensors:39"/device:CPU:0*
T0*
_output_shapes
:2
Identity_39?
AssignVariableOp_39AssignVariableOpassignvariableop_39_adam_beta_1Identity_39:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_39n
Identity_40IdentityRestoreV2:tensors:40"/device:CPU:0*
T0*
_output_shapes
:2
Identity_40?
AssignVariableOp_40AssignVariableOpassignvariableop_40_adam_beta_2Identity_40:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_40n
Identity_41IdentityRestoreV2:tensors:41"/device:CPU:0*
T0*
_output_shapes
:2
Identity_41?
AssignVariableOp_41AssignVariableOpassignvariableop_41_adam_decayIdentity_41:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_41n
Identity_42IdentityRestoreV2:tensors:42"/device:CPU:0*
T0*
_output_shapes
:2
Identity_42?
AssignVariableOp_42AssignVariableOp&assignvariableop_42_adam_learning_rateIdentity_42:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_42n
Identity_43IdentityRestoreV2:tensors:43"/device:CPU:0*
T0*
_output_shapes
:2
Identity_43?
AssignVariableOp_43AssignVariableOpassignvariableop_43_totalIdentity_43:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_43n
Identity_44IdentityRestoreV2:tensors:44"/device:CPU:0*
T0*
_output_shapes
:2
Identity_44?
AssignVariableOp_44AssignVariableOpassignvariableop_44_countIdentity_44:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_44n
Identity_45IdentityRestoreV2:tensors:45"/device:CPU:0*
T0*
_output_shapes
:2
Identity_45?
AssignVariableOp_45AssignVariableOpassignvariableop_45_total_1Identity_45:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_45n
Identity_46IdentityRestoreV2:tensors:46"/device:CPU:0*
T0*
_output_shapes
:2
Identity_46?
AssignVariableOp_46AssignVariableOpassignvariableop_46_count_1Identity_46:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_46n
Identity_47IdentityRestoreV2:tensors:47"/device:CPU:0*
T0*
_output_shapes
:2
Identity_47?
AssignVariableOp_47AssignVariableOp+assignvariableop_47_adam_conv2d_68_kernel_mIdentity_47:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_47n
Identity_48IdentityRestoreV2:tensors:48"/device:CPU:0*
T0*
_output_shapes
:2
Identity_48?
AssignVariableOp_48AssignVariableOp)assignvariableop_48_adam_conv2d_68_bias_mIdentity_48:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_48n
Identity_49IdentityRestoreV2:tensors:49"/device:CPU:0*
T0*
_output_shapes
:2
Identity_49?
AssignVariableOp_49AssignVariableOp8assignvariableop_49_adam_batch_normalization_102_gamma_mIdentity_49:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_49n
Identity_50IdentityRestoreV2:tensors:50"/device:CPU:0*
T0*
_output_shapes
:2
Identity_50?
AssignVariableOp_50AssignVariableOp7assignvariableop_50_adam_batch_normalization_102_beta_mIdentity_50:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_50n
Identity_51IdentityRestoreV2:tensors:51"/device:CPU:0*
T0*
_output_shapes
:2
Identity_51?
AssignVariableOp_51AssignVariableOp+assignvariableop_51_adam_conv2d_69_kernel_mIdentity_51:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_51n
Identity_52IdentityRestoreV2:tensors:52"/device:CPU:0*
T0*
_output_shapes
:2
Identity_52?
AssignVariableOp_52AssignVariableOp)assignvariableop_52_adam_conv2d_69_bias_mIdentity_52:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_52n
Identity_53IdentityRestoreV2:tensors:53"/device:CPU:0*
T0*
_output_shapes
:2
Identity_53?
AssignVariableOp_53AssignVariableOp8assignvariableop_53_adam_batch_normalization_103_gamma_mIdentity_53:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_53n
Identity_54IdentityRestoreV2:tensors:54"/device:CPU:0*
T0*
_output_shapes
:2
Identity_54?
AssignVariableOp_54AssignVariableOp7assignvariableop_54_adam_batch_normalization_103_beta_mIdentity_54:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_54n
Identity_55IdentityRestoreV2:tensors:55"/device:CPU:0*
T0*
_output_shapes
:2
Identity_55?
AssignVariableOp_55AssignVariableOp+assignvariableop_55_adam_conv2d_70_kernel_mIdentity_55:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_55n
Identity_56IdentityRestoreV2:tensors:56"/device:CPU:0*
T0*
_output_shapes
:2
Identity_56?
AssignVariableOp_56AssignVariableOp)assignvariableop_56_adam_conv2d_70_bias_mIdentity_56:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_56n
Identity_57IdentityRestoreV2:tensors:57"/device:CPU:0*
T0*
_output_shapes
:2
Identity_57?
AssignVariableOp_57AssignVariableOp8assignvariableop_57_adam_batch_normalization_104_gamma_mIdentity_57:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_57n
Identity_58IdentityRestoreV2:tensors:58"/device:CPU:0*
T0*
_output_shapes
:2
Identity_58?
AssignVariableOp_58AssignVariableOp7assignvariableop_58_adam_batch_normalization_104_beta_mIdentity_58:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_58n
Identity_59IdentityRestoreV2:tensors:59"/device:CPU:0*
T0*
_output_shapes
:2
Identity_59?
AssignVariableOp_59AssignVariableOp+assignvariableop_59_adam_conv2d_71_kernel_mIdentity_59:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_59n
Identity_60IdentityRestoreV2:tensors:60"/device:CPU:0*
T0*
_output_shapes
:2
Identity_60?
AssignVariableOp_60AssignVariableOp)assignvariableop_60_adam_conv2d_71_bias_mIdentity_60:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_60n
Identity_61IdentityRestoreV2:tensors:61"/device:CPU:0*
T0*
_output_shapes
:2
Identity_61?
AssignVariableOp_61AssignVariableOp8assignvariableop_61_adam_batch_normalization_105_gamma_mIdentity_61:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_61n
Identity_62IdentityRestoreV2:tensors:62"/device:CPU:0*
T0*
_output_shapes
:2
Identity_62?
AssignVariableOp_62AssignVariableOp7assignvariableop_62_adam_batch_normalization_105_beta_mIdentity_62:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_62n
Identity_63IdentityRestoreV2:tensors:63"/device:CPU:0*
T0*
_output_shapes
:2
Identity_63?
AssignVariableOp_63AssignVariableOp*assignvariableop_63_adam_dense_51_kernel_mIdentity_63:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_63n
Identity_64IdentityRestoreV2:tensors:64"/device:CPU:0*
T0*
_output_shapes
:2
Identity_64?
AssignVariableOp_64AssignVariableOp(assignvariableop_64_adam_dense_51_bias_mIdentity_64:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_64n
Identity_65IdentityRestoreV2:tensors:65"/device:CPU:0*
T0*
_output_shapes
:2
Identity_65?
AssignVariableOp_65AssignVariableOp8assignvariableop_65_adam_batch_normalization_106_gamma_mIdentity_65:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_65n
Identity_66IdentityRestoreV2:tensors:66"/device:CPU:0*
T0*
_output_shapes
:2
Identity_66?
AssignVariableOp_66AssignVariableOp7assignvariableop_66_adam_batch_normalization_106_beta_mIdentity_66:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_66n
Identity_67IdentityRestoreV2:tensors:67"/device:CPU:0*
T0*
_output_shapes
:2
Identity_67?
AssignVariableOp_67AssignVariableOp*assignvariableop_67_adam_dense_52_kernel_mIdentity_67:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_67n
Identity_68IdentityRestoreV2:tensors:68"/device:CPU:0*
T0*
_output_shapes
:2
Identity_68?
AssignVariableOp_68AssignVariableOp(assignvariableop_68_adam_dense_52_bias_mIdentity_68:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_68n
Identity_69IdentityRestoreV2:tensors:69"/device:CPU:0*
T0*
_output_shapes
:2
Identity_69?
AssignVariableOp_69AssignVariableOp8assignvariableop_69_adam_batch_normalization_107_gamma_mIdentity_69:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_69n
Identity_70IdentityRestoreV2:tensors:70"/device:CPU:0*
T0*
_output_shapes
:2
Identity_70?
AssignVariableOp_70AssignVariableOp7assignvariableop_70_adam_batch_normalization_107_beta_mIdentity_70:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_70n
Identity_71IdentityRestoreV2:tensors:71"/device:CPU:0*
T0*
_output_shapes
:2
Identity_71?
AssignVariableOp_71AssignVariableOp*assignvariableop_71_adam_dense_53_kernel_mIdentity_71:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_71n
Identity_72IdentityRestoreV2:tensors:72"/device:CPU:0*
T0*
_output_shapes
:2
Identity_72?
AssignVariableOp_72AssignVariableOp(assignvariableop_72_adam_dense_53_bias_mIdentity_72:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_72n
Identity_73IdentityRestoreV2:tensors:73"/device:CPU:0*
T0*
_output_shapes
:2
Identity_73?
AssignVariableOp_73AssignVariableOp+assignvariableop_73_adam_conv2d_68_kernel_vIdentity_73:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_73n
Identity_74IdentityRestoreV2:tensors:74"/device:CPU:0*
T0*
_output_shapes
:2
Identity_74?
AssignVariableOp_74AssignVariableOp)assignvariableop_74_adam_conv2d_68_bias_vIdentity_74:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_74n
Identity_75IdentityRestoreV2:tensors:75"/device:CPU:0*
T0*
_output_shapes
:2
Identity_75?
AssignVariableOp_75AssignVariableOp8assignvariableop_75_adam_batch_normalization_102_gamma_vIdentity_75:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_75n
Identity_76IdentityRestoreV2:tensors:76"/device:CPU:0*
T0*
_output_shapes
:2
Identity_76?
AssignVariableOp_76AssignVariableOp7assignvariableop_76_adam_batch_normalization_102_beta_vIdentity_76:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_76n
Identity_77IdentityRestoreV2:tensors:77"/device:CPU:0*
T0*
_output_shapes
:2
Identity_77?
AssignVariableOp_77AssignVariableOp+assignvariableop_77_adam_conv2d_69_kernel_vIdentity_77:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_77n
Identity_78IdentityRestoreV2:tensors:78"/device:CPU:0*
T0*
_output_shapes
:2
Identity_78?
AssignVariableOp_78AssignVariableOp)assignvariableop_78_adam_conv2d_69_bias_vIdentity_78:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_78n
Identity_79IdentityRestoreV2:tensors:79"/device:CPU:0*
T0*
_output_shapes
:2
Identity_79?
AssignVariableOp_79AssignVariableOp8assignvariableop_79_adam_batch_normalization_103_gamma_vIdentity_79:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_79n
Identity_80IdentityRestoreV2:tensors:80"/device:CPU:0*
T0*
_output_shapes
:2
Identity_80?
AssignVariableOp_80AssignVariableOp7assignvariableop_80_adam_batch_normalization_103_beta_vIdentity_80:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_80n
Identity_81IdentityRestoreV2:tensors:81"/device:CPU:0*
T0*
_output_shapes
:2
Identity_81?
AssignVariableOp_81AssignVariableOp+assignvariableop_81_adam_conv2d_70_kernel_vIdentity_81:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_81n
Identity_82IdentityRestoreV2:tensors:82"/device:CPU:0*
T0*
_output_shapes
:2
Identity_82?
AssignVariableOp_82AssignVariableOp)assignvariableop_82_adam_conv2d_70_bias_vIdentity_82:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_82n
Identity_83IdentityRestoreV2:tensors:83"/device:CPU:0*
T0*
_output_shapes
:2
Identity_83?
AssignVariableOp_83AssignVariableOp8assignvariableop_83_adam_batch_normalization_104_gamma_vIdentity_83:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_83n
Identity_84IdentityRestoreV2:tensors:84"/device:CPU:0*
T0*
_output_shapes
:2
Identity_84?
AssignVariableOp_84AssignVariableOp7assignvariableop_84_adam_batch_normalization_104_beta_vIdentity_84:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_84n
Identity_85IdentityRestoreV2:tensors:85"/device:CPU:0*
T0*
_output_shapes
:2
Identity_85?
AssignVariableOp_85AssignVariableOp+assignvariableop_85_adam_conv2d_71_kernel_vIdentity_85:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_85n
Identity_86IdentityRestoreV2:tensors:86"/device:CPU:0*
T0*
_output_shapes
:2
Identity_86?
AssignVariableOp_86AssignVariableOp)assignvariableop_86_adam_conv2d_71_bias_vIdentity_86:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_86n
Identity_87IdentityRestoreV2:tensors:87"/device:CPU:0*
T0*
_output_shapes
:2
Identity_87?
AssignVariableOp_87AssignVariableOp8assignvariableop_87_adam_batch_normalization_105_gamma_vIdentity_87:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_87n
Identity_88IdentityRestoreV2:tensors:88"/device:CPU:0*
T0*
_output_shapes
:2
Identity_88?
AssignVariableOp_88AssignVariableOp7assignvariableop_88_adam_batch_normalization_105_beta_vIdentity_88:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_88n
Identity_89IdentityRestoreV2:tensors:89"/device:CPU:0*
T0*
_output_shapes
:2
Identity_89?
AssignVariableOp_89AssignVariableOp*assignvariableop_89_adam_dense_51_kernel_vIdentity_89:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_89n
Identity_90IdentityRestoreV2:tensors:90"/device:CPU:0*
T0*
_output_shapes
:2
Identity_90?
AssignVariableOp_90AssignVariableOp(assignvariableop_90_adam_dense_51_bias_vIdentity_90:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_90n
Identity_91IdentityRestoreV2:tensors:91"/device:CPU:0*
T0*
_output_shapes
:2
Identity_91?
AssignVariableOp_91AssignVariableOp8assignvariableop_91_adam_batch_normalization_106_gamma_vIdentity_91:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_91n
Identity_92IdentityRestoreV2:tensors:92"/device:CPU:0*
T0*
_output_shapes
:2
Identity_92?
AssignVariableOp_92AssignVariableOp7assignvariableop_92_adam_batch_normalization_106_beta_vIdentity_92:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_92n
Identity_93IdentityRestoreV2:tensors:93"/device:CPU:0*
T0*
_output_shapes
:2
Identity_93?
AssignVariableOp_93AssignVariableOp*assignvariableop_93_adam_dense_52_kernel_vIdentity_93:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_93n
Identity_94IdentityRestoreV2:tensors:94"/device:CPU:0*
T0*
_output_shapes
:2
Identity_94?
AssignVariableOp_94AssignVariableOp(assignvariableop_94_adam_dense_52_bias_vIdentity_94:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_94n
Identity_95IdentityRestoreV2:tensors:95"/device:CPU:0*
T0*
_output_shapes
:2
Identity_95?
AssignVariableOp_95AssignVariableOp8assignvariableop_95_adam_batch_normalization_107_gamma_vIdentity_95:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_95n
Identity_96IdentityRestoreV2:tensors:96"/device:CPU:0*
T0*
_output_shapes
:2
Identity_96?
AssignVariableOp_96AssignVariableOp7assignvariableop_96_adam_batch_normalization_107_beta_vIdentity_96:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_96n
Identity_97IdentityRestoreV2:tensors:97"/device:CPU:0*
T0*
_output_shapes
:2
Identity_97?
AssignVariableOp_97AssignVariableOp*assignvariableop_97_adam_dense_53_kernel_vIdentity_97:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_97n
Identity_98IdentityRestoreV2:tensors:98"/device:CPU:0*
T0*
_output_shapes
:2
Identity_98?
AssignVariableOp_98AssignVariableOp(assignvariableop_98_adam_dense_53_bias_vIdentity_98:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_989
NoOpNoOp"/device:CPU:0*
_output_shapes
 2
NoOp?
Identity_99Identityfile_prefix^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_10^AssignVariableOp_11^AssignVariableOp_12^AssignVariableOp_13^AssignVariableOp_14^AssignVariableOp_15^AssignVariableOp_16^AssignVariableOp_17^AssignVariableOp_18^AssignVariableOp_19^AssignVariableOp_2^AssignVariableOp_20^AssignVariableOp_21^AssignVariableOp_22^AssignVariableOp_23^AssignVariableOp_24^AssignVariableOp_25^AssignVariableOp_26^AssignVariableOp_27^AssignVariableOp_28^AssignVariableOp_29^AssignVariableOp_3^AssignVariableOp_30^AssignVariableOp_31^AssignVariableOp_32^AssignVariableOp_33^AssignVariableOp_34^AssignVariableOp_35^AssignVariableOp_36^AssignVariableOp_37^AssignVariableOp_38^AssignVariableOp_39^AssignVariableOp_4^AssignVariableOp_40^AssignVariableOp_41^AssignVariableOp_42^AssignVariableOp_43^AssignVariableOp_44^AssignVariableOp_45^AssignVariableOp_46^AssignVariableOp_47^AssignVariableOp_48^AssignVariableOp_49^AssignVariableOp_5^AssignVariableOp_50^AssignVariableOp_51^AssignVariableOp_52^AssignVariableOp_53^AssignVariableOp_54^AssignVariableOp_55^AssignVariableOp_56^AssignVariableOp_57^AssignVariableOp_58^AssignVariableOp_59^AssignVariableOp_6^AssignVariableOp_60^AssignVariableOp_61^AssignVariableOp_62^AssignVariableOp_63^AssignVariableOp_64^AssignVariableOp_65^AssignVariableOp_66^AssignVariableOp_67^AssignVariableOp_68^AssignVariableOp_69^AssignVariableOp_7^AssignVariableOp_70^AssignVariableOp_71^AssignVariableOp_72^AssignVariableOp_73^AssignVariableOp_74^AssignVariableOp_75^AssignVariableOp_76^AssignVariableOp_77^AssignVariableOp_78^AssignVariableOp_79^AssignVariableOp_8^AssignVariableOp_80^AssignVariableOp_81^AssignVariableOp_82^AssignVariableOp_83^AssignVariableOp_84^AssignVariableOp_85^AssignVariableOp_86^AssignVariableOp_87^AssignVariableOp_88^AssignVariableOp_89^AssignVariableOp_9^AssignVariableOp_90^AssignVariableOp_91^AssignVariableOp_92^AssignVariableOp_93^AssignVariableOp_94^AssignVariableOp_95^AssignVariableOp_96^AssignVariableOp_97^AssignVariableOp_98^NoOp"/device:CPU:0*
T0*
_output_shapes
: 2
Identity_99?
Identity_100IdentityIdentity_99:output:0^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_10^AssignVariableOp_11^AssignVariableOp_12^AssignVariableOp_13^AssignVariableOp_14^AssignVariableOp_15^AssignVariableOp_16^AssignVariableOp_17^AssignVariableOp_18^AssignVariableOp_19^AssignVariableOp_2^AssignVariableOp_20^AssignVariableOp_21^AssignVariableOp_22^AssignVariableOp_23^AssignVariableOp_24^AssignVariableOp_25^AssignVariableOp_26^AssignVariableOp_27^AssignVariableOp_28^AssignVariableOp_29^AssignVariableOp_3^AssignVariableOp_30^AssignVariableOp_31^AssignVariableOp_32^AssignVariableOp_33^AssignVariableOp_34^AssignVariableOp_35^AssignVariableOp_36^AssignVariableOp_37^AssignVariableOp_38^AssignVariableOp_39^AssignVariableOp_4^AssignVariableOp_40^AssignVariableOp_41^AssignVariableOp_42^AssignVariableOp_43^AssignVariableOp_44^AssignVariableOp_45^AssignVariableOp_46^AssignVariableOp_47^AssignVariableOp_48^AssignVariableOp_49^AssignVariableOp_5^AssignVariableOp_50^AssignVariableOp_51^AssignVariableOp_52^AssignVariableOp_53^AssignVariableOp_54^AssignVariableOp_55^AssignVariableOp_56^AssignVariableOp_57^AssignVariableOp_58^AssignVariableOp_59^AssignVariableOp_6^AssignVariableOp_60^AssignVariableOp_61^AssignVariableOp_62^AssignVariableOp_63^AssignVariableOp_64^AssignVariableOp_65^AssignVariableOp_66^AssignVariableOp_67^AssignVariableOp_68^AssignVariableOp_69^AssignVariableOp_7^AssignVariableOp_70^AssignVariableOp_71^AssignVariableOp_72^AssignVariableOp_73^AssignVariableOp_74^AssignVariableOp_75^AssignVariableOp_76^AssignVariableOp_77^AssignVariableOp_78^AssignVariableOp_79^AssignVariableOp_8^AssignVariableOp_80^AssignVariableOp_81^AssignVariableOp_82^AssignVariableOp_83^AssignVariableOp_84^AssignVariableOp_85^AssignVariableOp_86^AssignVariableOp_87^AssignVariableOp_88^AssignVariableOp_89^AssignVariableOp_9^AssignVariableOp_90^AssignVariableOp_91^AssignVariableOp_92^AssignVariableOp_93^AssignVariableOp_94^AssignVariableOp_95^AssignVariableOp_96^AssignVariableOp_97^AssignVariableOp_98*
T0*
_output_shapes
: 2
Identity_100"%
identity_100Identity_100:output:0*?
_input_shapes?
?: :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::2$
AssignVariableOpAssignVariableOp2(
AssignVariableOp_1AssignVariableOp_12*
AssignVariableOp_10AssignVariableOp_102*
AssignVariableOp_11AssignVariableOp_112*
AssignVariableOp_12AssignVariableOp_122*
AssignVariableOp_13AssignVariableOp_132*
AssignVariableOp_14AssignVariableOp_142*
AssignVariableOp_15AssignVariableOp_152*
AssignVariableOp_16AssignVariableOp_162*
AssignVariableOp_17AssignVariableOp_172*
AssignVariableOp_18AssignVariableOp_182*
AssignVariableOp_19AssignVariableOp_192(
AssignVariableOp_2AssignVariableOp_22*
AssignVariableOp_20AssignVariableOp_202*
AssignVariableOp_21AssignVariableOp_212*
AssignVariableOp_22AssignVariableOp_222*
AssignVariableOp_23AssignVariableOp_232*
AssignVariableOp_24AssignVariableOp_242*
AssignVariableOp_25AssignVariableOp_252*
AssignVariableOp_26AssignVariableOp_262*
AssignVariableOp_27AssignVariableOp_272*
AssignVariableOp_28AssignVariableOp_282*
AssignVariableOp_29AssignVariableOp_292(
AssignVariableOp_3AssignVariableOp_32*
AssignVariableOp_30AssignVariableOp_302*
AssignVariableOp_31AssignVariableOp_312*
AssignVariableOp_32AssignVariableOp_322*
AssignVariableOp_33AssignVariableOp_332*
AssignVariableOp_34AssignVariableOp_342*
AssignVariableOp_35AssignVariableOp_352*
AssignVariableOp_36AssignVariableOp_362*
AssignVariableOp_37AssignVariableOp_372*
AssignVariableOp_38AssignVariableOp_382*
AssignVariableOp_39AssignVariableOp_392(
AssignVariableOp_4AssignVariableOp_42*
AssignVariableOp_40AssignVariableOp_402*
AssignVariableOp_41AssignVariableOp_412*
AssignVariableOp_42AssignVariableOp_422*
AssignVariableOp_43AssignVariableOp_432*
AssignVariableOp_44AssignVariableOp_442*
AssignVariableOp_45AssignVariableOp_452*
AssignVariableOp_46AssignVariableOp_462*
AssignVariableOp_47AssignVariableOp_472*
AssignVariableOp_48AssignVariableOp_482*
AssignVariableOp_49AssignVariableOp_492(
AssignVariableOp_5AssignVariableOp_52*
AssignVariableOp_50AssignVariableOp_502*
AssignVariableOp_51AssignVariableOp_512*
AssignVariableOp_52AssignVariableOp_522*
AssignVariableOp_53AssignVariableOp_532*
AssignVariableOp_54AssignVariableOp_542*
AssignVariableOp_55AssignVariableOp_552*
AssignVariableOp_56AssignVariableOp_562*
AssignVariableOp_57AssignVariableOp_572*
AssignVariableOp_58AssignVariableOp_582*
AssignVariableOp_59AssignVariableOp_592(
AssignVariableOp_6AssignVariableOp_62*
AssignVariableOp_60AssignVariableOp_602*
AssignVariableOp_61AssignVariableOp_612*
AssignVariableOp_62AssignVariableOp_622*
AssignVariableOp_63AssignVariableOp_632*
AssignVariableOp_64AssignVariableOp_642*
AssignVariableOp_65AssignVariableOp_652*
AssignVariableOp_66AssignVariableOp_662*
AssignVariableOp_67AssignVariableOp_672*
AssignVariableOp_68AssignVariableOp_682*
AssignVariableOp_69AssignVariableOp_692(
AssignVariableOp_7AssignVariableOp_72*
AssignVariableOp_70AssignVariableOp_702*
AssignVariableOp_71AssignVariableOp_712*
AssignVariableOp_72AssignVariableOp_722*
AssignVariableOp_73AssignVariableOp_732*
AssignVariableOp_74AssignVariableOp_742*
AssignVariableOp_75AssignVariableOp_752*
AssignVariableOp_76AssignVariableOp_762*
AssignVariableOp_77AssignVariableOp_772*
AssignVariableOp_78AssignVariableOp_782*
AssignVariableOp_79AssignVariableOp_792(
AssignVariableOp_8AssignVariableOp_82*
AssignVariableOp_80AssignVariableOp_802*
AssignVariableOp_81AssignVariableOp_812*
AssignVariableOp_82AssignVariableOp_822*
AssignVariableOp_83AssignVariableOp_832*
AssignVariableOp_84AssignVariableOp_842*
AssignVariableOp_85AssignVariableOp_852*
AssignVariableOp_86AssignVariableOp_862*
AssignVariableOp_87AssignVariableOp_872*
AssignVariableOp_88AssignVariableOp_882*
AssignVariableOp_89AssignVariableOp_892(
AssignVariableOp_9AssignVariableOp_92*
AssignVariableOp_90AssignVariableOp_902*
AssignVariableOp_91AssignVariableOp_912*
AssignVariableOp_92AssignVariableOp_922*
AssignVariableOp_93AssignVariableOp_932*
AssignVariableOp_94AssignVariableOp_942*
AssignVariableOp_95AssignVariableOp_952*
AssignVariableOp_96AssignVariableOp_962*
AssignVariableOp_97AssignVariableOp_972*
AssignVariableOp_98AssignVariableOp_98:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix
??
?'
!__inference__wrapped_model_582630
conv2d_68_input:
6sequential_17_conv2d_68_conv2d_readvariableop_resource;
7sequential_17_conv2d_68_biasadd_readvariableop_resourceA
=sequential_17_batch_normalization_102_readvariableop_resourceC
?sequential_17_batch_normalization_102_readvariableop_1_resourceR
Nsequential_17_batch_normalization_102_fusedbatchnormv3_readvariableop_resourceT
Psequential_17_batch_normalization_102_fusedbatchnormv3_readvariableop_1_resource:
6sequential_17_conv2d_69_conv2d_readvariableop_resource;
7sequential_17_conv2d_69_biasadd_readvariableop_resourceA
=sequential_17_batch_normalization_103_readvariableop_resourceC
?sequential_17_batch_normalization_103_readvariableop_1_resourceR
Nsequential_17_batch_normalization_103_fusedbatchnormv3_readvariableop_resourceT
Psequential_17_batch_normalization_103_fusedbatchnormv3_readvariableop_1_resource:
6sequential_17_conv2d_70_conv2d_readvariableop_resource;
7sequential_17_conv2d_70_biasadd_readvariableop_resourceA
=sequential_17_batch_normalization_104_readvariableop_resourceC
?sequential_17_batch_normalization_104_readvariableop_1_resourceR
Nsequential_17_batch_normalization_104_fusedbatchnormv3_readvariableop_resourceT
Psequential_17_batch_normalization_104_fusedbatchnormv3_readvariableop_1_resource:
6sequential_17_conv2d_71_conv2d_readvariableop_resource;
7sequential_17_conv2d_71_biasadd_readvariableop_resourceA
=sequential_17_batch_normalization_105_readvariableop_resourceC
?sequential_17_batch_normalization_105_readvariableop_1_resourceR
Nsequential_17_batch_normalization_105_fusedbatchnormv3_readvariableop_resourceT
Psequential_17_batch_normalization_105_fusedbatchnormv3_readvariableop_1_resource9
5sequential_17_dense_51_matmul_readvariableop_resource:
6sequential_17_dense_51_biasadd_readvariableop_resourceK
Gsequential_17_batch_normalization_106_batchnorm_readvariableop_resourceO
Ksequential_17_batch_normalization_106_batchnorm_mul_readvariableop_resourceM
Isequential_17_batch_normalization_106_batchnorm_readvariableop_1_resourceM
Isequential_17_batch_normalization_106_batchnorm_readvariableop_2_resource9
5sequential_17_dense_52_matmul_readvariableop_resource:
6sequential_17_dense_52_biasadd_readvariableop_resourceK
Gsequential_17_batch_normalization_107_batchnorm_readvariableop_resourceO
Ksequential_17_batch_normalization_107_batchnorm_mul_readvariableop_resourceM
Isequential_17_batch_normalization_107_batchnorm_readvariableop_1_resourceM
Isequential_17_batch_normalization_107_batchnorm_readvariableop_2_resource9
5sequential_17_dense_53_matmul_readvariableop_resource:
6sequential_17_dense_53_biasadd_readvariableop_resource
identity??Esequential_17/batch_normalization_102/FusedBatchNormV3/ReadVariableOp?Gsequential_17/batch_normalization_102/FusedBatchNormV3/ReadVariableOp_1?4sequential_17/batch_normalization_102/ReadVariableOp?6sequential_17/batch_normalization_102/ReadVariableOp_1?Esequential_17/batch_normalization_103/FusedBatchNormV3/ReadVariableOp?Gsequential_17/batch_normalization_103/FusedBatchNormV3/ReadVariableOp_1?4sequential_17/batch_normalization_103/ReadVariableOp?6sequential_17/batch_normalization_103/ReadVariableOp_1?Esequential_17/batch_normalization_104/FusedBatchNormV3/ReadVariableOp?Gsequential_17/batch_normalization_104/FusedBatchNormV3/ReadVariableOp_1?4sequential_17/batch_normalization_104/ReadVariableOp?6sequential_17/batch_normalization_104/ReadVariableOp_1?Esequential_17/batch_normalization_105/FusedBatchNormV3/ReadVariableOp?Gsequential_17/batch_normalization_105/FusedBatchNormV3/ReadVariableOp_1?4sequential_17/batch_normalization_105/ReadVariableOp?6sequential_17/batch_normalization_105/ReadVariableOp_1?>sequential_17/batch_normalization_106/batchnorm/ReadVariableOp?@sequential_17/batch_normalization_106/batchnorm/ReadVariableOp_1?@sequential_17/batch_normalization_106/batchnorm/ReadVariableOp_2?Bsequential_17/batch_normalization_106/batchnorm/mul/ReadVariableOp?>sequential_17/batch_normalization_107/batchnorm/ReadVariableOp?@sequential_17/batch_normalization_107/batchnorm/ReadVariableOp_1?@sequential_17/batch_normalization_107/batchnorm/ReadVariableOp_2?Bsequential_17/batch_normalization_107/batchnorm/mul/ReadVariableOp?.sequential_17/conv2d_68/BiasAdd/ReadVariableOp?-sequential_17/conv2d_68/Conv2D/ReadVariableOp?.sequential_17/conv2d_69/BiasAdd/ReadVariableOp?-sequential_17/conv2d_69/Conv2D/ReadVariableOp?.sequential_17/conv2d_70/BiasAdd/ReadVariableOp?-sequential_17/conv2d_70/Conv2D/ReadVariableOp?.sequential_17/conv2d_71/BiasAdd/ReadVariableOp?-sequential_17/conv2d_71/Conv2D/ReadVariableOp?-sequential_17/dense_51/BiasAdd/ReadVariableOp?,sequential_17/dense_51/MatMul/ReadVariableOp?-sequential_17/dense_52/BiasAdd/ReadVariableOp?,sequential_17/dense_52/MatMul/ReadVariableOp?-sequential_17/dense_53/BiasAdd/ReadVariableOp?,sequential_17/dense_53/MatMul/ReadVariableOp?
-sequential_17/conv2d_68/Conv2D/ReadVariableOpReadVariableOp6sequential_17_conv2d_68_conv2d_readvariableop_resource*&
_output_shapes
:<*
dtype02/
-sequential_17/conv2d_68/Conv2D/ReadVariableOp?
sequential_17/conv2d_68/Conv2DConv2Dconv2d_68_input5sequential_17/conv2d_68/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????<<<*
paddingVALID*
strides
2 
sequential_17/conv2d_68/Conv2D?
.sequential_17/conv2d_68/BiasAdd/ReadVariableOpReadVariableOp7sequential_17_conv2d_68_biasadd_readvariableop_resource*
_output_shapes
:<*
dtype020
.sequential_17/conv2d_68/BiasAdd/ReadVariableOp?
sequential_17/conv2d_68/BiasAddBiasAdd'sequential_17/conv2d_68/Conv2D:output:06sequential_17/conv2d_68/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????<<<2!
sequential_17/conv2d_68/BiasAdd?
!sequential_17/activation_119/ReluRelu(sequential_17/conv2d_68/BiasAdd:output:0*
T0*/
_output_shapes
:?????????<<<2#
!sequential_17/activation_119/Relu?
4sequential_17/batch_normalization_102/ReadVariableOpReadVariableOp=sequential_17_batch_normalization_102_readvariableop_resource*
_output_shapes
:<*
dtype026
4sequential_17/batch_normalization_102/ReadVariableOp?
6sequential_17/batch_normalization_102/ReadVariableOp_1ReadVariableOp?sequential_17_batch_normalization_102_readvariableop_1_resource*
_output_shapes
:<*
dtype028
6sequential_17/batch_normalization_102/ReadVariableOp_1?
Esequential_17/batch_normalization_102/FusedBatchNormV3/ReadVariableOpReadVariableOpNsequential_17_batch_normalization_102_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02G
Esequential_17/batch_normalization_102/FusedBatchNormV3/ReadVariableOp?
Gsequential_17/batch_normalization_102/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpPsequential_17_batch_normalization_102_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02I
Gsequential_17/batch_normalization_102/FusedBatchNormV3/ReadVariableOp_1?
6sequential_17/batch_normalization_102/FusedBatchNormV3FusedBatchNormV3/sequential_17/activation_119/Relu:activations:0<sequential_17/batch_normalization_102/ReadVariableOp:value:0>sequential_17/batch_normalization_102/ReadVariableOp_1:value:0Msequential_17/batch_normalization_102/FusedBatchNormV3/ReadVariableOp:value:0Osequential_17/batch_normalization_102/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????<<<:<:<:<:<:*
epsilon%o?:*
is_training( 28
6sequential_17/batch_normalization_102/FusedBatchNormV3?
-sequential_17/conv2d_69/Conv2D/ReadVariableOpReadVariableOp6sequential_17_conv2d_69_conv2d_readvariableop_resource*&
_output_shapes
:<<*
dtype02/
-sequential_17/conv2d_69/Conv2D/ReadVariableOp?
sequential_17/conv2d_69/Conv2DConv2D:sequential_17/batch_normalization_102/FusedBatchNormV3:y:05sequential_17/conv2d_69/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????88<*
paddingVALID*
strides
2 
sequential_17/conv2d_69/Conv2D?
.sequential_17/conv2d_69/BiasAdd/ReadVariableOpReadVariableOp7sequential_17_conv2d_69_biasadd_readvariableop_resource*
_output_shapes
:<*
dtype020
.sequential_17/conv2d_69/BiasAdd/ReadVariableOp?
sequential_17/conv2d_69/BiasAddBiasAdd'sequential_17/conv2d_69/Conv2D:output:06sequential_17/conv2d_69/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????88<2!
sequential_17/conv2d_69/BiasAdd?
!sequential_17/activation_120/ReluRelu(sequential_17/conv2d_69/BiasAdd:output:0*
T0*/
_output_shapes
:?????????88<2#
!sequential_17/activation_120/Relu?
4sequential_17/batch_normalization_103/ReadVariableOpReadVariableOp=sequential_17_batch_normalization_103_readvariableop_resource*
_output_shapes
:<*
dtype026
4sequential_17/batch_normalization_103/ReadVariableOp?
6sequential_17/batch_normalization_103/ReadVariableOp_1ReadVariableOp?sequential_17_batch_normalization_103_readvariableop_1_resource*
_output_shapes
:<*
dtype028
6sequential_17/batch_normalization_103/ReadVariableOp_1?
Esequential_17/batch_normalization_103/FusedBatchNormV3/ReadVariableOpReadVariableOpNsequential_17_batch_normalization_103_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02G
Esequential_17/batch_normalization_103/FusedBatchNormV3/ReadVariableOp?
Gsequential_17/batch_normalization_103/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpPsequential_17_batch_normalization_103_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02I
Gsequential_17/batch_normalization_103/FusedBatchNormV3/ReadVariableOp_1?
6sequential_17/batch_normalization_103/FusedBatchNormV3FusedBatchNormV3/sequential_17/activation_120/Relu:activations:0<sequential_17/batch_normalization_103/ReadVariableOp:value:0>sequential_17/batch_normalization_103/ReadVariableOp_1:value:0Msequential_17/batch_normalization_103/FusedBatchNormV3/ReadVariableOp:value:0Osequential_17/batch_normalization_103/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????88<:<:<:<:<:*
epsilon%o?:*
is_training( 28
6sequential_17/batch_normalization_103/FusedBatchNormV3?
&sequential_17/max_pooling2d_34/MaxPoolMaxPool:sequential_17/batch_normalization_103/FusedBatchNormV3:y:0*/
_output_shapes
:?????????<*
ksize
*
paddingVALID*
strides
2(
&sequential_17/max_pooling2d_34/MaxPool?
-sequential_17/conv2d_70/Conv2D/ReadVariableOpReadVariableOp6sequential_17_conv2d_70_conv2d_readvariableop_resource*&
_output_shapes
:<*
dtype02/
-sequential_17/conv2d_70/Conv2D/ReadVariableOp?
sequential_17/conv2d_70/Conv2DConv2D/sequential_17/max_pooling2d_34/MaxPool:output:05sequential_17/conv2d_70/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2 
sequential_17/conv2d_70/Conv2D?
.sequential_17/conv2d_70/BiasAdd/ReadVariableOpReadVariableOp7sequential_17_conv2d_70_biasadd_readvariableop_resource*
_output_shapes
:*
dtype020
.sequential_17/conv2d_70/BiasAdd/ReadVariableOp?
sequential_17/conv2d_70/BiasAddBiasAdd'sequential_17/conv2d_70/Conv2D:output:06sequential_17/conv2d_70/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????2!
sequential_17/conv2d_70/BiasAdd?
!sequential_17/activation_121/ReluRelu(sequential_17/conv2d_70/BiasAdd:output:0*
T0*/
_output_shapes
:?????????2#
!sequential_17/activation_121/Relu?
4sequential_17/batch_normalization_104/ReadVariableOpReadVariableOp=sequential_17_batch_normalization_104_readvariableop_resource*
_output_shapes
:*
dtype026
4sequential_17/batch_normalization_104/ReadVariableOp?
6sequential_17/batch_normalization_104/ReadVariableOp_1ReadVariableOp?sequential_17_batch_normalization_104_readvariableop_1_resource*
_output_shapes
:*
dtype028
6sequential_17/batch_normalization_104/ReadVariableOp_1?
Esequential_17/batch_normalization_104/FusedBatchNormV3/ReadVariableOpReadVariableOpNsequential_17_batch_normalization_104_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02G
Esequential_17/batch_normalization_104/FusedBatchNormV3/ReadVariableOp?
Gsequential_17/batch_normalization_104/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpPsequential_17_batch_normalization_104_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02I
Gsequential_17/batch_normalization_104/FusedBatchNormV3/ReadVariableOp_1?
6sequential_17/batch_normalization_104/FusedBatchNormV3FusedBatchNormV3/sequential_17/activation_121/Relu:activations:0<sequential_17/batch_normalization_104/ReadVariableOp:value:0>sequential_17/batch_normalization_104/ReadVariableOp_1:value:0Msequential_17/batch_normalization_104/FusedBatchNormV3/ReadVariableOp:value:0Osequential_17/batch_normalization_104/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
is_training( 28
6sequential_17/batch_normalization_104/FusedBatchNormV3?
-sequential_17/conv2d_71/Conv2D/ReadVariableOpReadVariableOp6sequential_17_conv2d_71_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02/
-sequential_17/conv2d_71/Conv2D/ReadVariableOp?
sequential_17/conv2d_71/Conv2DConv2D:sequential_17/batch_normalization_104/FusedBatchNormV3:y:05sequential_17/conv2d_71/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2 
sequential_17/conv2d_71/Conv2D?
.sequential_17/conv2d_71/BiasAdd/ReadVariableOpReadVariableOp7sequential_17_conv2d_71_biasadd_readvariableop_resource*
_output_shapes
:*
dtype020
.sequential_17/conv2d_71/BiasAdd/ReadVariableOp?
sequential_17/conv2d_71/BiasAddBiasAdd'sequential_17/conv2d_71/Conv2D:output:06sequential_17/conv2d_71/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????2!
sequential_17/conv2d_71/BiasAdd?
!sequential_17/activation_122/ReluRelu(sequential_17/conv2d_71/BiasAdd:output:0*
T0*/
_output_shapes
:?????????2#
!sequential_17/activation_122/Relu?
4sequential_17/batch_normalization_105/ReadVariableOpReadVariableOp=sequential_17_batch_normalization_105_readvariableop_resource*
_output_shapes
:*
dtype026
4sequential_17/batch_normalization_105/ReadVariableOp?
6sequential_17/batch_normalization_105/ReadVariableOp_1ReadVariableOp?sequential_17_batch_normalization_105_readvariableop_1_resource*
_output_shapes
:*
dtype028
6sequential_17/batch_normalization_105/ReadVariableOp_1?
Esequential_17/batch_normalization_105/FusedBatchNormV3/ReadVariableOpReadVariableOpNsequential_17_batch_normalization_105_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02G
Esequential_17/batch_normalization_105/FusedBatchNormV3/ReadVariableOp?
Gsequential_17/batch_normalization_105/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpPsequential_17_batch_normalization_105_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02I
Gsequential_17/batch_normalization_105/FusedBatchNormV3/ReadVariableOp_1?
6sequential_17/batch_normalization_105/FusedBatchNormV3FusedBatchNormV3/sequential_17/activation_122/Relu:activations:0<sequential_17/batch_normalization_105/ReadVariableOp:value:0>sequential_17/batch_normalization_105/ReadVariableOp_1:value:0Msequential_17/batch_normalization_105/FusedBatchNormV3/ReadVariableOp:value:0Osequential_17/batch_normalization_105/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
is_training( 28
6sequential_17/batch_normalization_105/FusedBatchNormV3?
&sequential_17/max_pooling2d_35/MaxPoolMaxPool:sequential_17/batch_normalization_105/FusedBatchNormV3:y:0*/
_output_shapes
:?????????*
ksize
*
paddingVALID*
strides
2(
&sequential_17/max_pooling2d_35/MaxPool?
!sequential_17/dropout_51/IdentityIdentity/sequential_17/max_pooling2d_35/MaxPool:output:0*
T0*/
_output_shapes
:?????????2#
!sequential_17/dropout_51/Identity?
sequential_17/flatten_34/ConstConst*
_output_shapes
:*
dtype0*
valueB"?????  2 
sequential_17/flatten_34/Const?
 sequential_17/flatten_34/ReshapeReshape*sequential_17/dropout_51/Identity:output:0'sequential_17/flatten_34/Const:output:0*
T0*(
_output_shapes
:??????????!2"
 sequential_17/flatten_34/Reshape?
,sequential_17/dense_51/MatMul/ReadVariableOpReadVariableOp5sequential_17_dense_51_matmul_readvariableop_resource* 
_output_shapes
:
?!?*
dtype02.
,sequential_17/dense_51/MatMul/ReadVariableOp?
sequential_17/dense_51/MatMulMatMul)sequential_17/flatten_34/Reshape:output:04sequential_17/dense_51/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
sequential_17/dense_51/MatMul?
-sequential_17/dense_51/BiasAdd/ReadVariableOpReadVariableOp6sequential_17_dense_51_biasadd_readvariableop_resource*
_output_shapes	
:?*
dtype02/
-sequential_17/dense_51/BiasAdd/ReadVariableOp?
sequential_17/dense_51/BiasAddBiasAdd'sequential_17/dense_51/MatMul:product:05sequential_17/dense_51/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2 
sequential_17/dense_51/BiasAdd?
!sequential_17/activation_123/ReluRelu'sequential_17/dense_51/BiasAdd:output:0*
T0*(
_output_shapes
:??????????2#
!sequential_17/activation_123/Relu?
>sequential_17/batch_normalization_106/batchnorm/ReadVariableOpReadVariableOpGsequential_17_batch_normalization_106_batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype02@
>sequential_17/batch_normalization_106/batchnorm/ReadVariableOp?
5sequential_17/batch_normalization_106/batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:27
5sequential_17/batch_normalization_106/batchnorm/add/y?
3sequential_17/batch_normalization_106/batchnorm/addAddV2Fsequential_17/batch_normalization_106/batchnorm/ReadVariableOp:value:0>sequential_17/batch_normalization_106/batchnorm/add/y:output:0*
T0*
_output_shapes	
:?25
3sequential_17/batch_normalization_106/batchnorm/add?
5sequential_17/batch_normalization_106/batchnorm/RsqrtRsqrt7sequential_17/batch_normalization_106/batchnorm/add:z:0*
T0*
_output_shapes	
:?27
5sequential_17/batch_normalization_106/batchnorm/Rsqrt?
Bsequential_17/batch_normalization_106/batchnorm/mul/ReadVariableOpReadVariableOpKsequential_17_batch_normalization_106_batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype02D
Bsequential_17/batch_normalization_106/batchnorm/mul/ReadVariableOp?
3sequential_17/batch_normalization_106/batchnorm/mulMul9sequential_17/batch_normalization_106/batchnorm/Rsqrt:y:0Jsequential_17/batch_normalization_106/batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?25
3sequential_17/batch_normalization_106/batchnorm/mul?
5sequential_17/batch_normalization_106/batchnorm/mul_1Mul/sequential_17/activation_123/Relu:activations:07sequential_17/batch_normalization_106/batchnorm/mul:z:0*
T0*(
_output_shapes
:??????????27
5sequential_17/batch_normalization_106/batchnorm/mul_1?
@sequential_17/batch_normalization_106/batchnorm/ReadVariableOp_1ReadVariableOpIsequential_17_batch_normalization_106_batchnorm_readvariableop_1_resource*
_output_shapes	
:?*
dtype02B
@sequential_17/batch_normalization_106/batchnorm/ReadVariableOp_1?
5sequential_17/batch_normalization_106/batchnorm/mul_2MulHsequential_17/batch_normalization_106/batchnorm/ReadVariableOp_1:value:07sequential_17/batch_normalization_106/batchnorm/mul:z:0*
T0*
_output_shapes	
:?27
5sequential_17/batch_normalization_106/batchnorm/mul_2?
@sequential_17/batch_normalization_106/batchnorm/ReadVariableOp_2ReadVariableOpIsequential_17_batch_normalization_106_batchnorm_readvariableop_2_resource*
_output_shapes	
:?*
dtype02B
@sequential_17/batch_normalization_106/batchnorm/ReadVariableOp_2?
3sequential_17/batch_normalization_106/batchnorm/subSubHsequential_17/batch_normalization_106/batchnorm/ReadVariableOp_2:value:09sequential_17/batch_normalization_106/batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?25
3sequential_17/batch_normalization_106/batchnorm/sub?
5sequential_17/batch_normalization_106/batchnorm/add_1AddV29sequential_17/batch_normalization_106/batchnorm/mul_1:z:07sequential_17/batch_normalization_106/batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????27
5sequential_17/batch_normalization_106/batchnorm/add_1?
!sequential_17/dropout_52/IdentityIdentity9sequential_17/batch_normalization_106/batchnorm/add_1:z:0*
T0*(
_output_shapes
:??????????2#
!sequential_17/dropout_52/Identity?
sequential_17/flatten_35/ConstConst*
_output_shapes
:*
dtype0*
valueB"????,  2 
sequential_17/flatten_35/Const?
 sequential_17/flatten_35/ReshapeReshape*sequential_17/dropout_52/Identity:output:0'sequential_17/flatten_35/Const:output:0*
T0*(
_output_shapes
:??????????2"
 sequential_17/flatten_35/Reshape?
,sequential_17/dense_52/MatMul/ReadVariableOpReadVariableOp5sequential_17_dense_52_matmul_readvariableop_resource* 
_output_shapes
:
??*
dtype02.
,sequential_17/dense_52/MatMul/ReadVariableOp?
sequential_17/dense_52/MatMulMatMul)sequential_17/flatten_35/Reshape:output:04sequential_17/dense_52/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
sequential_17/dense_52/MatMul?
-sequential_17/dense_52/BiasAdd/ReadVariableOpReadVariableOp6sequential_17_dense_52_biasadd_readvariableop_resource*
_output_shapes	
:?*
dtype02/
-sequential_17/dense_52/BiasAdd/ReadVariableOp?
sequential_17/dense_52/BiasAddBiasAdd'sequential_17/dense_52/MatMul:product:05sequential_17/dense_52/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2 
sequential_17/dense_52/BiasAdd?
!sequential_17/activation_124/ReluRelu'sequential_17/dense_52/BiasAdd:output:0*
T0*(
_output_shapes
:??????????2#
!sequential_17/activation_124/Relu?
>sequential_17/batch_normalization_107/batchnorm/ReadVariableOpReadVariableOpGsequential_17_batch_normalization_107_batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype02@
>sequential_17/batch_normalization_107/batchnorm/ReadVariableOp?
5sequential_17/batch_normalization_107/batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:27
5sequential_17/batch_normalization_107/batchnorm/add/y?
3sequential_17/batch_normalization_107/batchnorm/addAddV2Fsequential_17/batch_normalization_107/batchnorm/ReadVariableOp:value:0>sequential_17/batch_normalization_107/batchnorm/add/y:output:0*
T0*
_output_shapes	
:?25
3sequential_17/batch_normalization_107/batchnorm/add?
5sequential_17/batch_normalization_107/batchnorm/RsqrtRsqrt7sequential_17/batch_normalization_107/batchnorm/add:z:0*
T0*
_output_shapes	
:?27
5sequential_17/batch_normalization_107/batchnorm/Rsqrt?
Bsequential_17/batch_normalization_107/batchnorm/mul/ReadVariableOpReadVariableOpKsequential_17_batch_normalization_107_batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype02D
Bsequential_17/batch_normalization_107/batchnorm/mul/ReadVariableOp?
3sequential_17/batch_normalization_107/batchnorm/mulMul9sequential_17/batch_normalization_107/batchnorm/Rsqrt:y:0Jsequential_17/batch_normalization_107/batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?25
3sequential_17/batch_normalization_107/batchnorm/mul?
5sequential_17/batch_normalization_107/batchnorm/mul_1Mul/sequential_17/activation_124/Relu:activations:07sequential_17/batch_normalization_107/batchnorm/mul:z:0*
T0*(
_output_shapes
:??????????27
5sequential_17/batch_normalization_107/batchnorm/mul_1?
@sequential_17/batch_normalization_107/batchnorm/ReadVariableOp_1ReadVariableOpIsequential_17_batch_normalization_107_batchnorm_readvariableop_1_resource*
_output_shapes	
:?*
dtype02B
@sequential_17/batch_normalization_107/batchnorm/ReadVariableOp_1?
5sequential_17/batch_normalization_107/batchnorm/mul_2MulHsequential_17/batch_normalization_107/batchnorm/ReadVariableOp_1:value:07sequential_17/batch_normalization_107/batchnorm/mul:z:0*
T0*
_output_shapes	
:?27
5sequential_17/batch_normalization_107/batchnorm/mul_2?
@sequential_17/batch_normalization_107/batchnorm/ReadVariableOp_2ReadVariableOpIsequential_17_batch_normalization_107_batchnorm_readvariableop_2_resource*
_output_shapes	
:?*
dtype02B
@sequential_17/batch_normalization_107/batchnorm/ReadVariableOp_2?
3sequential_17/batch_normalization_107/batchnorm/subSubHsequential_17/batch_normalization_107/batchnorm/ReadVariableOp_2:value:09sequential_17/batch_normalization_107/batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?25
3sequential_17/batch_normalization_107/batchnorm/sub?
5sequential_17/batch_normalization_107/batchnorm/add_1AddV29sequential_17/batch_normalization_107/batchnorm/mul_1:z:07sequential_17/batch_normalization_107/batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????27
5sequential_17/batch_normalization_107/batchnorm/add_1?
!sequential_17/dropout_53/IdentityIdentity9sequential_17/batch_normalization_107/batchnorm/add_1:z:0*
T0*(
_output_shapes
:??????????2#
!sequential_17/dropout_53/Identity?
,sequential_17/dense_53/MatMul/ReadVariableOpReadVariableOp5sequential_17_dense_53_matmul_readvariableop_resource*
_output_shapes
:	?*
dtype02.
,sequential_17/dense_53/MatMul/ReadVariableOp?
sequential_17/dense_53/MatMulMatMul*sequential_17/dropout_53/Identity:output:04sequential_17/dense_53/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
sequential_17/dense_53/MatMul?
-sequential_17/dense_53/BiasAdd/ReadVariableOpReadVariableOp6sequential_17_dense_53_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02/
-sequential_17/dense_53/BiasAdd/ReadVariableOp?
sequential_17/dense_53/BiasAddBiasAdd'sequential_17/dense_53/MatMul:product:05sequential_17/dense_53/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2 
sequential_17/dense_53/BiasAdd?
$sequential_17/activation_125/SoftmaxSoftmax'sequential_17/dense_53/BiasAdd:output:0*
T0*'
_output_shapes
:?????????2&
$sequential_17/activation_125/Softmax?
IdentityIdentity.sequential_17/activation_125/Softmax:softmax:0F^sequential_17/batch_normalization_102/FusedBatchNormV3/ReadVariableOpH^sequential_17/batch_normalization_102/FusedBatchNormV3/ReadVariableOp_15^sequential_17/batch_normalization_102/ReadVariableOp7^sequential_17/batch_normalization_102/ReadVariableOp_1F^sequential_17/batch_normalization_103/FusedBatchNormV3/ReadVariableOpH^sequential_17/batch_normalization_103/FusedBatchNormV3/ReadVariableOp_15^sequential_17/batch_normalization_103/ReadVariableOp7^sequential_17/batch_normalization_103/ReadVariableOp_1F^sequential_17/batch_normalization_104/FusedBatchNormV3/ReadVariableOpH^sequential_17/batch_normalization_104/FusedBatchNormV3/ReadVariableOp_15^sequential_17/batch_normalization_104/ReadVariableOp7^sequential_17/batch_normalization_104/ReadVariableOp_1F^sequential_17/batch_normalization_105/FusedBatchNormV3/ReadVariableOpH^sequential_17/batch_normalization_105/FusedBatchNormV3/ReadVariableOp_15^sequential_17/batch_normalization_105/ReadVariableOp7^sequential_17/batch_normalization_105/ReadVariableOp_1?^sequential_17/batch_normalization_106/batchnorm/ReadVariableOpA^sequential_17/batch_normalization_106/batchnorm/ReadVariableOp_1A^sequential_17/batch_normalization_106/batchnorm/ReadVariableOp_2C^sequential_17/batch_normalization_106/batchnorm/mul/ReadVariableOp?^sequential_17/batch_normalization_107/batchnorm/ReadVariableOpA^sequential_17/batch_normalization_107/batchnorm/ReadVariableOp_1A^sequential_17/batch_normalization_107/batchnorm/ReadVariableOp_2C^sequential_17/batch_normalization_107/batchnorm/mul/ReadVariableOp/^sequential_17/conv2d_68/BiasAdd/ReadVariableOp.^sequential_17/conv2d_68/Conv2D/ReadVariableOp/^sequential_17/conv2d_69/BiasAdd/ReadVariableOp.^sequential_17/conv2d_69/Conv2D/ReadVariableOp/^sequential_17/conv2d_70/BiasAdd/ReadVariableOp.^sequential_17/conv2d_70/Conv2D/ReadVariableOp/^sequential_17/conv2d_71/BiasAdd/ReadVariableOp.^sequential_17/conv2d_71/Conv2D/ReadVariableOp.^sequential_17/dense_51/BiasAdd/ReadVariableOp-^sequential_17/dense_51/MatMul/ReadVariableOp.^sequential_17/dense_52/BiasAdd/ReadVariableOp-^sequential_17/dense_52/MatMul/ReadVariableOp.^sequential_17/dense_53/BiasAdd/ReadVariableOp-^sequential_17/dense_53/MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::2?
Esequential_17/batch_normalization_102/FusedBatchNormV3/ReadVariableOpEsequential_17/batch_normalization_102/FusedBatchNormV3/ReadVariableOp2?
Gsequential_17/batch_normalization_102/FusedBatchNormV3/ReadVariableOp_1Gsequential_17/batch_normalization_102/FusedBatchNormV3/ReadVariableOp_12l
4sequential_17/batch_normalization_102/ReadVariableOp4sequential_17/batch_normalization_102/ReadVariableOp2p
6sequential_17/batch_normalization_102/ReadVariableOp_16sequential_17/batch_normalization_102/ReadVariableOp_12?
Esequential_17/batch_normalization_103/FusedBatchNormV3/ReadVariableOpEsequential_17/batch_normalization_103/FusedBatchNormV3/ReadVariableOp2?
Gsequential_17/batch_normalization_103/FusedBatchNormV3/ReadVariableOp_1Gsequential_17/batch_normalization_103/FusedBatchNormV3/ReadVariableOp_12l
4sequential_17/batch_normalization_103/ReadVariableOp4sequential_17/batch_normalization_103/ReadVariableOp2p
6sequential_17/batch_normalization_103/ReadVariableOp_16sequential_17/batch_normalization_103/ReadVariableOp_12?
Esequential_17/batch_normalization_104/FusedBatchNormV3/ReadVariableOpEsequential_17/batch_normalization_104/FusedBatchNormV3/ReadVariableOp2?
Gsequential_17/batch_normalization_104/FusedBatchNormV3/ReadVariableOp_1Gsequential_17/batch_normalization_104/FusedBatchNormV3/ReadVariableOp_12l
4sequential_17/batch_normalization_104/ReadVariableOp4sequential_17/batch_normalization_104/ReadVariableOp2p
6sequential_17/batch_normalization_104/ReadVariableOp_16sequential_17/batch_normalization_104/ReadVariableOp_12?
Esequential_17/batch_normalization_105/FusedBatchNormV3/ReadVariableOpEsequential_17/batch_normalization_105/FusedBatchNormV3/ReadVariableOp2?
Gsequential_17/batch_normalization_105/FusedBatchNormV3/ReadVariableOp_1Gsequential_17/batch_normalization_105/FusedBatchNormV3/ReadVariableOp_12l
4sequential_17/batch_normalization_105/ReadVariableOp4sequential_17/batch_normalization_105/ReadVariableOp2p
6sequential_17/batch_normalization_105/ReadVariableOp_16sequential_17/batch_normalization_105/ReadVariableOp_12?
>sequential_17/batch_normalization_106/batchnorm/ReadVariableOp>sequential_17/batch_normalization_106/batchnorm/ReadVariableOp2?
@sequential_17/batch_normalization_106/batchnorm/ReadVariableOp_1@sequential_17/batch_normalization_106/batchnorm/ReadVariableOp_12?
@sequential_17/batch_normalization_106/batchnorm/ReadVariableOp_2@sequential_17/batch_normalization_106/batchnorm/ReadVariableOp_22?
Bsequential_17/batch_normalization_106/batchnorm/mul/ReadVariableOpBsequential_17/batch_normalization_106/batchnorm/mul/ReadVariableOp2?
>sequential_17/batch_normalization_107/batchnorm/ReadVariableOp>sequential_17/batch_normalization_107/batchnorm/ReadVariableOp2?
@sequential_17/batch_normalization_107/batchnorm/ReadVariableOp_1@sequential_17/batch_normalization_107/batchnorm/ReadVariableOp_12?
@sequential_17/batch_normalization_107/batchnorm/ReadVariableOp_2@sequential_17/batch_normalization_107/batchnorm/ReadVariableOp_22?
Bsequential_17/batch_normalization_107/batchnorm/mul/ReadVariableOpBsequential_17/batch_normalization_107/batchnorm/mul/ReadVariableOp2`
.sequential_17/conv2d_68/BiasAdd/ReadVariableOp.sequential_17/conv2d_68/BiasAdd/ReadVariableOp2^
-sequential_17/conv2d_68/Conv2D/ReadVariableOp-sequential_17/conv2d_68/Conv2D/ReadVariableOp2`
.sequential_17/conv2d_69/BiasAdd/ReadVariableOp.sequential_17/conv2d_69/BiasAdd/ReadVariableOp2^
-sequential_17/conv2d_69/Conv2D/ReadVariableOp-sequential_17/conv2d_69/Conv2D/ReadVariableOp2`
.sequential_17/conv2d_70/BiasAdd/ReadVariableOp.sequential_17/conv2d_70/BiasAdd/ReadVariableOp2^
-sequential_17/conv2d_70/Conv2D/ReadVariableOp-sequential_17/conv2d_70/Conv2D/ReadVariableOp2`
.sequential_17/conv2d_71/BiasAdd/ReadVariableOp.sequential_17/conv2d_71/BiasAdd/ReadVariableOp2^
-sequential_17/conv2d_71/Conv2D/ReadVariableOp-sequential_17/conv2d_71/Conv2D/ReadVariableOp2^
-sequential_17/dense_51/BiasAdd/ReadVariableOp-sequential_17/dense_51/BiasAdd/ReadVariableOp2\
,sequential_17/dense_51/MatMul/ReadVariableOp,sequential_17/dense_51/MatMul/ReadVariableOp2^
-sequential_17/dense_52/BiasAdd/ReadVariableOp-sequential_17/dense_52/BiasAdd/ReadVariableOp2\
,sequential_17/dense_52/MatMul/ReadVariableOp,sequential_17/dense_52/MatMul/ReadVariableOp2^
-sequential_17/dense_53/BiasAdd/ReadVariableOp-sequential_17/dense_53/BiasAdd/ReadVariableOp2\
,sequential_17/dense_53/MatMul/ReadVariableOp,sequential_17/dense_53/MatMul/ReadVariableOp:` \
/
_output_shapes
:?????????@@
)
_user_specified_nameconv2d_68_input
?
e
F__inference_dropout_51_layer_call_and_return_conditional_losses_585848

inputs
identity?c
dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *UU??2
dropout/Const{
dropout/MulMulinputsdropout/Const:output:0*
T0*/
_output_shapes
:?????????2
dropout/MulT
dropout/ShapeShapeinputs*
T0*
_output_shapes
:2
dropout/Shape?
$dropout/random_uniform/RandomUniformRandomUniformdropout/Shape:output:0*
T0*/
_output_shapes
:?????????*
dtype02&
$dropout/random_uniform/RandomUniformu
dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *???>2
dropout/GreaterEqual/y?
dropout/GreaterEqualGreaterEqual-dropout/random_uniform/RandomUniform:output:0dropout/GreaterEqual/y:output:0*
T0*/
_output_shapes
:?????????2
dropout/GreaterEqual?
dropout/CastCastdropout/GreaterEqual:z:0*

DstT0*

SrcT0
*/
_output_shapes
:?????????2
dropout/Cast?
dropout/Mul_1Muldropout/Mul:z:0dropout/Cast:y:0*
T0*/
_output_shapes
:?????????2
dropout/Mul_1m
IdentityIdentitydropout/Mul_1:z:0*
T0*/
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
d
F__inference_dropout_53_layer_call_and_return_conditional_losses_584057

inputs

identity_1[
IdentityIdentityinputs*
T0*(
_output_shapes
:??????????2

Identityj

Identity_1IdentityIdentity:output:0*
T0*(
_output_shapes
:??????????2

Identity_1"!

identity_1Identity_1:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
d
F__inference_dropout_52_layer_call_and_return_conditional_losses_583939

inputs

identity_1[
IdentityIdentityinputs*
T0*(
_output_shapes
:??????????2

Identityj

Identity_1IdentityIdentity:output:0*
T0*(
_output_shapes
:??????????2

Identity_1"!

identity_1Identity_1:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_582692

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??AssignNewValue?AssignNewValue_1?FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:<*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:<*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*]
_output_shapesK
I:+???????????????????????????<:<:<:<:<:*
epsilon%o?:*
exponential_avg_factor%
?#<2
FusedBatchNormV3?
AssignNewValueAssignVariableOp(fusedbatchnormv3_readvariableop_resourceFusedBatchNormV3:batch_mean:0 ^FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*;
_class1
/-loc:@FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02
AssignNewValue?
AssignNewValue_1AssignVariableOp*fusedbatchnormv3_readvariableop_1_resource!FusedBatchNormV3:batch_variance:0"^FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*=
_class3
1/loc:@FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02
AssignNewValue_1?
IdentityIdentityFusedBatchNormV3:y:0^AssignNewValue^AssignNewValue_1 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*A
_output_shapes/
-:+???????????????????????????<2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????<::::2 
AssignNewValueAssignNewValue2$
AssignNewValue_1AssignNewValue_12B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:i e
A
_output_shapes/
-:+???????????????????????????<
 
_user_specified_nameinputs
?	
?
D__inference_dense_52_layer_call_and_return_conditional_losses_586033

inputs"
matmul_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?MatMul/ReadVariableOp?
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource* 
_output_shapes
:
??*
dtype02
MatMul/ReadVariableOpt
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
MatMul?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes	
:?*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*/
_input_shapes
:??????????::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?	
?
D__inference_dense_51_layer_call_and_return_conditional_losses_585884

inputs"
matmul_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?MatMul/ReadVariableOp?
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource* 
_output_shapes
:
?!?*
dtype02
MatMul/ReadVariableOpt
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
MatMul?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes	
:?*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*/
_input_shapes
:??????????!::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:P L
(
_output_shapes
:??????????!
 
_user_specified_nameinputs
?
?
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_583047

inputs
readvariableop_resource
readvariableop_1_resource,
(fusedbatchnormv3_readvariableop_resource.
*fusedbatchnormv3_readvariableop_1_resource
identity??FusedBatchNormV3/ReadVariableOp?!FusedBatchNormV3/ReadVariableOp_1?ReadVariableOp?ReadVariableOp_1t
ReadVariableOpReadVariableOpreadvariableop_resource*
_output_shapes
:*
dtype02
ReadVariableOpz
ReadVariableOp_1ReadVariableOpreadvariableop_1_resource*
_output_shapes
:*
dtype02
ReadVariableOp_1?
FusedBatchNormV3/ReadVariableOpReadVariableOp(fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02!
FusedBatchNormV3/ReadVariableOp?
!FusedBatchNormV3/ReadVariableOp_1ReadVariableOp*fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02#
!FusedBatchNormV3/ReadVariableOp_1?
FusedBatchNormV3FusedBatchNormV3inputsReadVariableOp:value:0ReadVariableOp_1:value:0'FusedBatchNormV3/ReadVariableOp:value:0)FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*]
_output_shapesK
I:+???????????????????????????:::::*
epsilon%o?:*
is_training( 2
FusedBatchNormV3?
IdentityIdentityFusedBatchNormV3:y:0 ^FusedBatchNormV3/ReadVariableOp"^FusedBatchNormV3/ReadVariableOp_1^ReadVariableOp^ReadVariableOp_1*
T0*A
_output_shapes/
-:+???????????????????????????2

Identity"
identityIdentity:output:0*P
_input_shapes?
=:+???????????????????????????::::2B
FusedBatchNormV3/ReadVariableOpFusedBatchNormV3/ReadVariableOp2F
!FusedBatchNormV3/ReadVariableOp_1!FusedBatchNormV3/ReadVariableOp_12 
ReadVariableOpReadVariableOp2$
ReadVariableOp_1ReadVariableOp_1:i e
A
_output_shapes/
-:+???????????????????????????
 
_user_specified_nameinputs
?
K
/__inference_activation_119_layer_call_fn_585237

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<<<* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_119_layer_call_and_return_conditional_losses_5833852
PartitionedCallt
IdentityIdentityPartitionedCall:output:0*
T0*/
_output_shapes
:?????????<<<2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????<<<:W S
/
_output_shapes
:?????????<<<
 
_user_specified_nameinputs
?
M
1__inference_max_pooling2d_35_layer_call_fn_583070

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *J
_output_shapes8
6:4????????????????????????????????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *U
fPRN
L__inference_max_pooling2d_35_layer_call_and_return_conditional_losses_5830642
PartitionedCall?
IdentityIdentityPartitionedCall:output:0*
T0*J
_output_shapes8
6:4????????????????????????????????????2

Identity"
identityIdentity:output:0*I
_input_shapes8
6:4????????????????????????????????????:r n
J
_output_shapes8
6:4????????????????????????????????????
 
_user_specified_nameinputs
??
?#
I__inference_sequential_17_layer_call_and_return_conditional_losses_584896

inputs,
(conv2d_68_conv2d_readvariableop_resource-
)conv2d_68_biasadd_readvariableop_resource3
/batch_normalization_102_readvariableop_resource5
1batch_normalization_102_readvariableop_1_resourceD
@batch_normalization_102_fusedbatchnormv3_readvariableop_resourceF
Bbatch_normalization_102_fusedbatchnormv3_readvariableop_1_resource,
(conv2d_69_conv2d_readvariableop_resource-
)conv2d_69_biasadd_readvariableop_resource3
/batch_normalization_103_readvariableop_resource5
1batch_normalization_103_readvariableop_1_resourceD
@batch_normalization_103_fusedbatchnormv3_readvariableop_resourceF
Bbatch_normalization_103_fusedbatchnormv3_readvariableop_1_resource,
(conv2d_70_conv2d_readvariableop_resource-
)conv2d_70_biasadd_readvariableop_resource3
/batch_normalization_104_readvariableop_resource5
1batch_normalization_104_readvariableop_1_resourceD
@batch_normalization_104_fusedbatchnormv3_readvariableop_resourceF
Bbatch_normalization_104_fusedbatchnormv3_readvariableop_1_resource,
(conv2d_71_conv2d_readvariableop_resource-
)conv2d_71_biasadd_readvariableop_resource3
/batch_normalization_105_readvariableop_resource5
1batch_normalization_105_readvariableop_1_resourceD
@batch_normalization_105_fusedbatchnormv3_readvariableop_resourceF
Bbatch_normalization_105_fusedbatchnormv3_readvariableop_1_resource+
'dense_51_matmul_readvariableop_resource,
(dense_51_biasadd_readvariableop_resource2
.batch_normalization_106_assignmovingavg_5848074
0batch_normalization_106_assignmovingavg_1_584813A
=batch_normalization_106_batchnorm_mul_readvariableop_resource=
9batch_normalization_106_batchnorm_readvariableop_resource+
'dense_52_matmul_readvariableop_resource,
(dense_52_biasadd_readvariableop_resource2
.batch_normalization_107_assignmovingavg_5848564
0batch_normalization_107_assignmovingavg_1_584862A
=batch_normalization_107_batchnorm_mul_readvariableop_resource=
9batch_normalization_107_batchnorm_readvariableop_resource+
'dense_53_matmul_readvariableop_resource,
(dense_53_biasadd_readvariableop_resource
identity??&batch_normalization_102/AssignNewValue?(batch_normalization_102/AssignNewValue_1?7batch_normalization_102/FusedBatchNormV3/ReadVariableOp?9batch_normalization_102/FusedBatchNormV3/ReadVariableOp_1?&batch_normalization_102/ReadVariableOp?(batch_normalization_102/ReadVariableOp_1?&batch_normalization_103/AssignNewValue?(batch_normalization_103/AssignNewValue_1?7batch_normalization_103/FusedBatchNormV3/ReadVariableOp?9batch_normalization_103/FusedBatchNormV3/ReadVariableOp_1?&batch_normalization_103/ReadVariableOp?(batch_normalization_103/ReadVariableOp_1?&batch_normalization_104/AssignNewValue?(batch_normalization_104/AssignNewValue_1?7batch_normalization_104/FusedBatchNormV3/ReadVariableOp?9batch_normalization_104/FusedBatchNormV3/ReadVariableOp_1?&batch_normalization_104/ReadVariableOp?(batch_normalization_104/ReadVariableOp_1?&batch_normalization_105/AssignNewValue?(batch_normalization_105/AssignNewValue_1?7batch_normalization_105/FusedBatchNormV3/ReadVariableOp?9batch_normalization_105/FusedBatchNormV3/ReadVariableOp_1?&batch_normalization_105/ReadVariableOp?(batch_normalization_105/ReadVariableOp_1?;batch_normalization_106/AssignMovingAvg/AssignSubVariableOp?6batch_normalization_106/AssignMovingAvg/ReadVariableOp?=batch_normalization_106/AssignMovingAvg_1/AssignSubVariableOp?8batch_normalization_106/AssignMovingAvg_1/ReadVariableOp?0batch_normalization_106/batchnorm/ReadVariableOp?4batch_normalization_106/batchnorm/mul/ReadVariableOp?;batch_normalization_107/AssignMovingAvg/AssignSubVariableOp?6batch_normalization_107/AssignMovingAvg/ReadVariableOp?=batch_normalization_107/AssignMovingAvg_1/AssignSubVariableOp?8batch_normalization_107/AssignMovingAvg_1/ReadVariableOp?0batch_normalization_107/batchnorm/ReadVariableOp?4batch_normalization_107/batchnorm/mul/ReadVariableOp? conv2d_68/BiasAdd/ReadVariableOp?conv2d_68/Conv2D/ReadVariableOp? conv2d_69/BiasAdd/ReadVariableOp?conv2d_69/Conv2D/ReadVariableOp? conv2d_70/BiasAdd/ReadVariableOp?conv2d_70/Conv2D/ReadVariableOp? conv2d_71/BiasAdd/ReadVariableOp?conv2d_71/Conv2D/ReadVariableOp?dense_51/BiasAdd/ReadVariableOp?dense_51/MatMul/ReadVariableOp?dense_52/BiasAdd/ReadVariableOp?dense_52/MatMul/ReadVariableOp?dense_53/BiasAdd/ReadVariableOp?dense_53/MatMul/ReadVariableOp?
conv2d_68/Conv2D/ReadVariableOpReadVariableOp(conv2d_68_conv2d_readvariableop_resource*&
_output_shapes
:<*
dtype02!
conv2d_68/Conv2D/ReadVariableOp?
conv2d_68/Conv2DConv2Dinputs'conv2d_68/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????<<<*
paddingVALID*
strides
2
conv2d_68/Conv2D?
 conv2d_68/BiasAdd/ReadVariableOpReadVariableOp)conv2d_68_biasadd_readvariableop_resource*
_output_shapes
:<*
dtype02"
 conv2d_68/BiasAdd/ReadVariableOp?
conv2d_68/BiasAddBiasAddconv2d_68/Conv2D:output:0(conv2d_68/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????<<<2
conv2d_68/BiasAdd?
activation_119/ReluReluconv2d_68/BiasAdd:output:0*
T0*/
_output_shapes
:?????????<<<2
activation_119/Relu?
&batch_normalization_102/ReadVariableOpReadVariableOp/batch_normalization_102_readvariableop_resource*
_output_shapes
:<*
dtype02(
&batch_normalization_102/ReadVariableOp?
(batch_normalization_102/ReadVariableOp_1ReadVariableOp1batch_normalization_102_readvariableop_1_resource*
_output_shapes
:<*
dtype02*
(batch_normalization_102/ReadVariableOp_1?
7batch_normalization_102/FusedBatchNormV3/ReadVariableOpReadVariableOp@batch_normalization_102_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype029
7batch_normalization_102/FusedBatchNormV3/ReadVariableOp?
9batch_normalization_102/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpBbatch_normalization_102_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02;
9batch_normalization_102/FusedBatchNormV3/ReadVariableOp_1?
(batch_normalization_102/FusedBatchNormV3FusedBatchNormV3!activation_119/Relu:activations:0.batch_normalization_102/ReadVariableOp:value:00batch_normalization_102/ReadVariableOp_1:value:0?batch_normalization_102/FusedBatchNormV3/ReadVariableOp:value:0Abatch_normalization_102/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????<<<:<:<:<:<:*
epsilon%o?:*
exponential_avg_factor%
?#<2*
(batch_normalization_102/FusedBatchNormV3?
&batch_normalization_102/AssignNewValueAssignVariableOp@batch_normalization_102_fusedbatchnormv3_readvariableop_resource5batch_normalization_102/FusedBatchNormV3:batch_mean:08^batch_normalization_102/FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*S
_classI
GEloc:@batch_normalization_102/FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02(
&batch_normalization_102/AssignNewValue?
(batch_normalization_102/AssignNewValue_1AssignVariableOpBbatch_normalization_102_fusedbatchnormv3_readvariableop_1_resource9batch_normalization_102/FusedBatchNormV3:batch_variance:0:^batch_normalization_102/FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*U
_classK
IGloc:@batch_normalization_102/FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02*
(batch_normalization_102/AssignNewValue_1?
conv2d_69/Conv2D/ReadVariableOpReadVariableOp(conv2d_69_conv2d_readvariableop_resource*&
_output_shapes
:<<*
dtype02!
conv2d_69/Conv2D/ReadVariableOp?
conv2d_69/Conv2DConv2D,batch_normalization_102/FusedBatchNormV3:y:0'conv2d_69/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????88<*
paddingVALID*
strides
2
conv2d_69/Conv2D?
 conv2d_69/BiasAdd/ReadVariableOpReadVariableOp)conv2d_69_biasadd_readvariableop_resource*
_output_shapes
:<*
dtype02"
 conv2d_69/BiasAdd/ReadVariableOp?
conv2d_69/BiasAddBiasAddconv2d_69/Conv2D:output:0(conv2d_69/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????88<2
conv2d_69/BiasAdd?
activation_120/ReluReluconv2d_69/BiasAdd:output:0*
T0*/
_output_shapes
:?????????88<2
activation_120/Relu?
&batch_normalization_103/ReadVariableOpReadVariableOp/batch_normalization_103_readvariableop_resource*
_output_shapes
:<*
dtype02(
&batch_normalization_103/ReadVariableOp?
(batch_normalization_103/ReadVariableOp_1ReadVariableOp1batch_normalization_103_readvariableop_1_resource*
_output_shapes
:<*
dtype02*
(batch_normalization_103/ReadVariableOp_1?
7batch_normalization_103/FusedBatchNormV3/ReadVariableOpReadVariableOp@batch_normalization_103_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype029
7batch_normalization_103/FusedBatchNormV3/ReadVariableOp?
9batch_normalization_103/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpBbatch_normalization_103_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02;
9batch_normalization_103/FusedBatchNormV3/ReadVariableOp_1?
(batch_normalization_103/FusedBatchNormV3FusedBatchNormV3!activation_120/Relu:activations:0.batch_normalization_103/ReadVariableOp:value:00batch_normalization_103/ReadVariableOp_1:value:0?batch_normalization_103/FusedBatchNormV3/ReadVariableOp:value:0Abatch_normalization_103/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????88<:<:<:<:<:*
epsilon%o?:*
exponential_avg_factor%
?#<2*
(batch_normalization_103/FusedBatchNormV3?
&batch_normalization_103/AssignNewValueAssignVariableOp@batch_normalization_103_fusedbatchnormv3_readvariableop_resource5batch_normalization_103/FusedBatchNormV3:batch_mean:08^batch_normalization_103/FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*S
_classI
GEloc:@batch_normalization_103/FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02(
&batch_normalization_103/AssignNewValue?
(batch_normalization_103/AssignNewValue_1AssignVariableOpBbatch_normalization_103_fusedbatchnormv3_readvariableop_1_resource9batch_normalization_103/FusedBatchNormV3:batch_variance:0:^batch_normalization_103/FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*U
_classK
IGloc:@batch_normalization_103/FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02*
(batch_normalization_103/AssignNewValue_1?
max_pooling2d_34/MaxPoolMaxPool,batch_normalization_103/FusedBatchNormV3:y:0*/
_output_shapes
:?????????<*
ksize
*
paddingVALID*
strides
2
max_pooling2d_34/MaxPool?
conv2d_70/Conv2D/ReadVariableOpReadVariableOp(conv2d_70_conv2d_readvariableop_resource*&
_output_shapes
:<*
dtype02!
conv2d_70/Conv2D/ReadVariableOp?
conv2d_70/Conv2DConv2D!max_pooling2d_34/MaxPool:output:0'conv2d_70/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2
conv2d_70/Conv2D?
 conv2d_70/BiasAdd/ReadVariableOpReadVariableOp)conv2d_70_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02"
 conv2d_70/BiasAdd/ReadVariableOp?
conv2d_70/BiasAddBiasAddconv2d_70/Conv2D:output:0(conv2d_70/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????2
conv2d_70/BiasAdd?
activation_121/ReluReluconv2d_70/BiasAdd:output:0*
T0*/
_output_shapes
:?????????2
activation_121/Relu?
&batch_normalization_104/ReadVariableOpReadVariableOp/batch_normalization_104_readvariableop_resource*
_output_shapes
:*
dtype02(
&batch_normalization_104/ReadVariableOp?
(batch_normalization_104/ReadVariableOp_1ReadVariableOp1batch_normalization_104_readvariableop_1_resource*
_output_shapes
:*
dtype02*
(batch_normalization_104/ReadVariableOp_1?
7batch_normalization_104/FusedBatchNormV3/ReadVariableOpReadVariableOp@batch_normalization_104_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype029
7batch_normalization_104/FusedBatchNormV3/ReadVariableOp?
9batch_normalization_104/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpBbatch_normalization_104_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02;
9batch_normalization_104/FusedBatchNormV3/ReadVariableOp_1?
(batch_normalization_104/FusedBatchNormV3FusedBatchNormV3!activation_121/Relu:activations:0.batch_normalization_104/ReadVariableOp:value:00batch_normalization_104/ReadVariableOp_1:value:0?batch_normalization_104/FusedBatchNormV3/ReadVariableOp:value:0Abatch_normalization_104/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
exponential_avg_factor%
?#<2*
(batch_normalization_104/FusedBatchNormV3?
&batch_normalization_104/AssignNewValueAssignVariableOp@batch_normalization_104_fusedbatchnormv3_readvariableop_resource5batch_normalization_104/FusedBatchNormV3:batch_mean:08^batch_normalization_104/FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*S
_classI
GEloc:@batch_normalization_104/FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02(
&batch_normalization_104/AssignNewValue?
(batch_normalization_104/AssignNewValue_1AssignVariableOpBbatch_normalization_104_fusedbatchnormv3_readvariableop_1_resource9batch_normalization_104/FusedBatchNormV3:batch_variance:0:^batch_normalization_104/FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*U
_classK
IGloc:@batch_normalization_104/FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02*
(batch_normalization_104/AssignNewValue_1?
conv2d_71/Conv2D/ReadVariableOpReadVariableOp(conv2d_71_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02!
conv2d_71/Conv2D/ReadVariableOp?
conv2d_71/Conv2DConv2D,batch_normalization_104/FusedBatchNormV3:y:0'conv2d_71/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2
conv2d_71/Conv2D?
 conv2d_71/BiasAdd/ReadVariableOpReadVariableOp)conv2d_71_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02"
 conv2d_71/BiasAdd/ReadVariableOp?
conv2d_71/BiasAddBiasAddconv2d_71/Conv2D:output:0(conv2d_71/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????2
conv2d_71/BiasAdd?
activation_122/ReluReluconv2d_71/BiasAdd:output:0*
T0*/
_output_shapes
:?????????2
activation_122/Relu?
&batch_normalization_105/ReadVariableOpReadVariableOp/batch_normalization_105_readvariableop_resource*
_output_shapes
:*
dtype02(
&batch_normalization_105/ReadVariableOp?
(batch_normalization_105/ReadVariableOp_1ReadVariableOp1batch_normalization_105_readvariableop_1_resource*
_output_shapes
:*
dtype02*
(batch_normalization_105/ReadVariableOp_1?
7batch_normalization_105/FusedBatchNormV3/ReadVariableOpReadVariableOp@batch_normalization_105_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype029
7batch_normalization_105/FusedBatchNormV3/ReadVariableOp?
9batch_normalization_105/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpBbatch_normalization_105_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02;
9batch_normalization_105/FusedBatchNormV3/ReadVariableOp_1?
(batch_normalization_105/FusedBatchNormV3FusedBatchNormV3!activation_122/Relu:activations:0.batch_normalization_105/ReadVariableOp:value:00batch_normalization_105/ReadVariableOp_1:value:0?batch_normalization_105/FusedBatchNormV3/ReadVariableOp:value:0Abatch_normalization_105/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
exponential_avg_factor%
?#<2*
(batch_normalization_105/FusedBatchNormV3?
&batch_normalization_105/AssignNewValueAssignVariableOp@batch_normalization_105_fusedbatchnormv3_readvariableop_resource5batch_normalization_105/FusedBatchNormV3:batch_mean:08^batch_normalization_105/FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*S
_classI
GEloc:@batch_normalization_105/FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02(
&batch_normalization_105/AssignNewValue?
(batch_normalization_105/AssignNewValue_1AssignVariableOpBbatch_normalization_105_fusedbatchnormv3_readvariableop_1_resource9batch_normalization_105/FusedBatchNormV3:batch_variance:0:^batch_normalization_105/FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*U
_classK
IGloc:@batch_normalization_105/FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02*
(batch_normalization_105/AssignNewValue_1?
max_pooling2d_35/MaxPoolMaxPool,batch_normalization_105/FusedBatchNormV3:y:0*/
_output_shapes
:?????????*
ksize
*
paddingVALID*
strides
2
max_pooling2d_35/MaxPooly
dropout_51/dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *UU??2
dropout_51/dropout/Const?
dropout_51/dropout/MulMul!max_pooling2d_35/MaxPool:output:0!dropout_51/dropout/Const:output:0*
T0*/
_output_shapes
:?????????2
dropout_51/dropout/Mul?
dropout_51/dropout/ShapeShape!max_pooling2d_35/MaxPool:output:0*
T0*
_output_shapes
:2
dropout_51/dropout/Shape?
/dropout_51/dropout/random_uniform/RandomUniformRandomUniform!dropout_51/dropout/Shape:output:0*
T0*/
_output_shapes
:?????????*
dtype021
/dropout_51/dropout/random_uniform/RandomUniform?
!dropout_51/dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *???>2#
!dropout_51/dropout/GreaterEqual/y?
dropout_51/dropout/GreaterEqualGreaterEqual8dropout_51/dropout/random_uniform/RandomUniform:output:0*dropout_51/dropout/GreaterEqual/y:output:0*
T0*/
_output_shapes
:?????????2!
dropout_51/dropout/GreaterEqual?
dropout_51/dropout/CastCast#dropout_51/dropout/GreaterEqual:z:0*

DstT0*

SrcT0
*/
_output_shapes
:?????????2
dropout_51/dropout/Cast?
dropout_51/dropout/Mul_1Muldropout_51/dropout/Mul:z:0dropout_51/dropout/Cast:y:0*
T0*/
_output_shapes
:?????????2
dropout_51/dropout/Mul_1u
flatten_34/ConstConst*
_output_shapes
:*
dtype0*
valueB"?????  2
flatten_34/Const?
flatten_34/ReshapeReshapedropout_51/dropout/Mul_1:z:0flatten_34/Const:output:0*
T0*(
_output_shapes
:??????????!2
flatten_34/Reshape?
dense_51/MatMul/ReadVariableOpReadVariableOp'dense_51_matmul_readvariableop_resource* 
_output_shapes
:
?!?*
dtype02 
dense_51/MatMul/ReadVariableOp?
dense_51/MatMulMatMulflatten_34/Reshape:output:0&dense_51/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
dense_51/MatMul?
dense_51/BiasAdd/ReadVariableOpReadVariableOp(dense_51_biasadd_readvariableop_resource*
_output_shapes	
:?*
dtype02!
dense_51/BiasAdd/ReadVariableOp?
dense_51/BiasAddBiasAdddense_51/MatMul:product:0'dense_51/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
dense_51/BiasAdd?
activation_123/ReluReludense_51/BiasAdd:output:0*
T0*(
_output_shapes
:??????????2
activation_123/Relu?
6batch_normalization_106/moments/mean/reduction_indicesConst*
_output_shapes
:*
dtype0*
valueB: 28
6batch_normalization_106/moments/mean/reduction_indices?
$batch_normalization_106/moments/meanMean!activation_123/Relu:activations:0?batch_normalization_106/moments/mean/reduction_indices:output:0*
T0*
_output_shapes
:	?*
	keep_dims(2&
$batch_normalization_106/moments/mean?
,batch_normalization_106/moments/StopGradientStopGradient-batch_normalization_106/moments/mean:output:0*
T0*
_output_shapes
:	?2.
,batch_normalization_106/moments/StopGradient?
1batch_normalization_106/moments/SquaredDifferenceSquaredDifference!activation_123/Relu:activations:05batch_normalization_106/moments/StopGradient:output:0*
T0*(
_output_shapes
:??????????23
1batch_normalization_106/moments/SquaredDifference?
:batch_normalization_106/moments/variance/reduction_indicesConst*
_output_shapes
:*
dtype0*
valueB: 2<
:batch_normalization_106/moments/variance/reduction_indices?
(batch_normalization_106/moments/varianceMean5batch_normalization_106/moments/SquaredDifference:z:0Cbatch_normalization_106/moments/variance/reduction_indices:output:0*
T0*
_output_shapes
:	?*
	keep_dims(2*
(batch_normalization_106/moments/variance?
'batch_normalization_106/moments/SqueezeSqueeze-batch_normalization_106/moments/mean:output:0*
T0*
_output_shapes	
:?*
squeeze_dims
 2)
'batch_normalization_106/moments/Squeeze?
)batch_normalization_106/moments/Squeeze_1Squeeze1batch_normalization_106/moments/variance:output:0*
T0*
_output_shapes	
:?*
squeeze_dims
 2+
)batch_normalization_106/moments/Squeeze_1?
-batch_normalization_106/AssignMovingAvg/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*A
_class7
53loc:@batch_normalization_106/AssignMovingAvg/584807*
_output_shapes
: *
dtype0*
valueB
 *
?#<2/
-batch_normalization_106/AssignMovingAvg/decay?
6batch_normalization_106/AssignMovingAvg/ReadVariableOpReadVariableOp.batch_normalization_106_assignmovingavg_584807*
_output_shapes	
:?*
dtype028
6batch_normalization_106/AssignMovingAvg/ReadVariableOp?
+batch_normalization_106/AssignMovingAvg/subSub>batch_normalization_106/AssignMovingAvg/ReadVariableOp:value:00batch_normalization_106/moments/Squeeze:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*A
_class7
53loc:@batch_normalization_106/AssignMovingAvg/584807*
_output_shapes	
:?2-
+batch_normalization_106/AssignMovingAvg/sub?
+batch_normalization_106/AssignMovingAvg/mulMul/batch_normalization_106/AssignMovingAvg/sub:z:06batch_normalization_106/AssignMovingAvg/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*A
_class7
53loc:@batch_normalization_106/AssignMovingAvg/584807*
_output_shapes	
:?2-
+batch_normalization_106/AssignMovingAvg/mul?
;batch_normalization_106/AssignMovingAvg/AssignSubVariableOpAssignSubVariableOp.batch_normalization_106_assignmovingavg_584807/batch_normalization_106/AssignMovingAvg/mul:z:07^batch_normalization_106/AssignMovingAvg/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*A
_class7
53loc:@batch_normalization_106/AssignMovingAvg/584807*
_output_shapes
 *
dtype02=
;batch_normalization_106/AssignMovingAvg/AssignSubVariableOp?
/batch_normalization_106/AssignMovingAvg_1/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*C
_class9
75loc:@batch_normalization_106/AssignMovingAvg_1/584813*
_output_shapes
: *
dtype0*
valueB
 *
?#<21
/batch_normalization_106/AssignMovingAvg_1/decay?
8batch_normalization_106/AssignMovingAvg_1/ReadVariableOpReadVariableOp0batch_normalization_106_assignmovingavg_1_584813*
_output_shapes	
:?*
dtype02:
8batch_normalization_106/AssignMovingAvg_1/ReadVariableOp?
-batch_normalization_106/AssignMovingAvg_1/subSub@batch_normalization_106/AssignMovingAvg_1/ReadVariableOp:value:02batch_normalization_106/moments/Squeeze_1:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*C
_class9
75loc:@batch_normalization_106/AssignMovingAvg_1/584813*
_output_shapes	
:?2/
-batch_normalization_106/AssignMovingAvg_1/sub?
-batch_normalization_106/AssignMovingAvg_1/mulMul1batch_normalization_106/AssignMovingAvg_1/sub:z:08batch_normalization_106/AssignMovingAvg_1/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*C
_class9
75loc:@batch_normalization_106/AssignMovingAvg_1/584813*
_output_shapes	
:?2/
-batch_normalization_106/AssignMovingAvg_1/mul?
=batch_normalization_106/AssignMovingAvg_1/AssignSubVariableOpAssignSubVariableOp0batch_normalization_106_assignmovingavg_1_5848131batch_normalization_106/AssignMovingAvg_1/mul:z:09^batch_normalization_106/AssignMovingAvg_1/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*C
_class9
75loc:@batch_normalization_106/AssignMovingAvg_1/584813*
_output_shapes
 *
dtype02?
=batch_normalization_106/AssignMovingAvg_1/AssignSubVariableOp?
'batch_normalization_106/batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:2)
'batch_normalization_106/batchnorm/add/y?
%batch_normalization_106/batchnorm/addAddV22batch_normalization_106/moments/Squeeze_1:output:00batch_normalization_106/batchnorm/add/y:output:0*
T0*
_output_shapes	
:?2'
%batch_normalization_106/batchnorm/add?
'batch_normalization_106/batchnorm/RsqrtRsqrt)batch_normalization_106/batchnorm/add:z:0*
T0*
_output_shapes	
:?2)
'batch_normalization_106/batchnorm/Rsqrt?
4batch_normalization_106/batchnorm/mul/ReadVariableOpReadVariableOp=batch_normalization_106_batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype026
4batch_normalization_106/batchnorm/mul/ReadVariableOp?
%batch_normalization_106/batchnorm/mulMul+batch_normalization_106/batchnorm/Rsqrt:y:0<batch_normalization_106/batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?2'
%batch_normalization_106/batchnorm/mul?
'batch_normalization_106/batchnorm/mul_1Mul!activation_123/Relu:activations:0)batch_normalization_106/batchnorm/mul:z:0*
T0*(
_output_shapes
:??????????2)
'batch_normalization_106/batchnorm/mul_1?
'batch_normalization_106/batchnorm/mul_2Mul0batch_normalization_106/moments/Squeeze:output:0)batch_normalization_106/batchnorm/mul:z:0*
T0*
_output_shapes	
:?2)
'batch_normalization_106/batchnorm/mul_2?
0batch_normalization_106/batchnorm/ReadVariableOpReadVariableOp9batch_normalization_106_batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype022
0batch_normalization_106/batchnorm/ReadVariableOp?
%batch_normalization_106/batchnorm/subSub8batch_normalization_106/batchnorm/ReadVariableOp:value:0+batch_normalization_106/batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?2'
%batch_normalization_106/batchnorm/sub?
'batch_normalization_106/batchnorm/add_1AddV2+batch_normalization_106/batchnorm/mul_1:z:0)batch_normalization_106/batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????2)
'batch_normalization_106/batchnorm/add_1y
dropout_52/dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *UU??2
dropout_52/dropout/Const?
dropout_52/dropout/MulMul+batch_normalization_106/batchnorm/add_1:z:0!dropout_52/dropout/Const:output:0*
T0*(
_output_shapes
:??????????2
dropout_52/dropout/Mul?
dropout_52/dropout/ShapeShape+batch_normalization_106/batchnorm/add_1:z:0*
T0*
_output_shapes
:2
dropout_52/dropout/Shape?
/dropout_52/dropout/random_uniform/RandomUniformRandomUniform!dropout_52/dropout/Shape:output:0*
T0*(
_output_shapes
:??????????*
dtype021
/dropout_52/dropout/random_uniform/RandomUniform?
!dropout_52/dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *???>2#
!dropout_52/dropout/GreaterEqual/y?
dropout_52/dropout/GreaterEqualGreaterEqual8dropout_52/dropout/random_uniform/RandomUniform:output:0*dropout_52/dropout/GreaterEqual/y:output:0*
T0*(
_output_shapes
:??????????2!
dropout_52/dropout/GreaterEqual?
dropout_52/dropout/CastCast#dropout_52/dropout/GreaterEqual:z:0*

DstT0*

SrcT0
*(
_output_shapes
:??????????2
dropout_52/dropout/Cast?
dropout_52/dropout/Mul_1Muldropout_52/dropout/Mul:z:0dropout_52/dropout/Cast:y:0*
T0*(
_output_shapes
:??????????2
dropout_52/dropout/Mul_1u
flatten_35/ConstConst*
_output_shapes
:*
dtype0*
valueB"????,  2
flatten_35/Const?
flatten_35/ReshapeReshapedropout_52/dropout/Mul_1:z:0flatten_35/Const:output:0*
T0*(
_output_shapes
:??????????2
flatten_35/Reshape?
dense_52/MatMul/ReadVariableOpReadVariableOp'dense_52_matmul_readvariableop_resource* 
_output_shapes
:
??*
dtype02 
dense_52/MatMul/ReadVariableOp?
dense_52/MatMulMatMulflatten_35/Reshape:output:0&dense_52/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
dense_52/MatMul?
dense_52/BiasAdd/ReadVariableOpReadVariableOp(dense_52_biasadd_readvariableop_resource*
_output_shapes	
:?*
dtype02!
dense_52/BiasAdd/ReadVariableOp?
dense_52/BiasAddBiasAdddense_52/MatMul:product:0'dense_52/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
dense_52/BiasAdd?
activation_124/ReluReludense_52/BiasAdd:output:0*
T0*(
_output_shapes
:??????????2
activation_124/Relu?
6batch_normalization_107/moments/mean/reduction_indicesConst*
_output_shapes
:*
dtype0*
valueB: 28
6batch_normalization_107/moments/mean/reduction_indices?
$batch_normalization_107/moments/meanMean!activation_124/Relu:activations:0?batch_normalization_107/moments/mean/reduction_indices:output:0*
T0*
_output_shapes
:	?*
	keep_dims(2&
$batch_normalization_107/moments/mean?
,batch_normalization_107/moments/StopGradientStopGradient-batch_normalization_107/moments/mean:output:0*
T0*
_output_shapes
:	?2.
,batch_normalization_107/moments/StopGradient?
1batch_normalization_107/moments/SquaredDifferenceSquaredDifference!activation_124/Relu:activations:05batch_normalization_107/moments/StopGradient:output:0*
T0*(
_output_shapes
:??????????23
1batch_normalization_107/moments/SquaredDifference?
:batch_normalization_107/moments/variance/reduction_indicesConst*
_output_shapes
:*
dtype0*
valueB: 2<
:batch_normalization_107/moments/variance/reduction_indices?
(batch_normalization_107/moments/varianceMean5batch_normalization_107/moments/SquaredDifference:z:0Cbatch_normalization_107/moments/variance/reduction_indices:output:0*
T0*
_output_shapes
:	?*
	keep_dims(2*
(batch_normalization_107/moments/variance?
'batch_normalization_107/moments/SqueezeSqueeze-batch_normalization_107/moments/mean:output:0*
T0*
_output_shapes	
:?*
squeeze_dims
 2)
'batch_normalization_107/moments/Squeeze?
)batch_normalization_107/moments/Squeeze_1Squeeze1batch_normalization_107/moments/variance:output:0*
T0*
_output_shapes	
:?*
squeeze_dims
 2+
)batch_normalization_107/moments/Squeeze_1?
-batch_normalization_107/AssignMovingAvg/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*A
_class7
53loc:@batch_normalization_107/AssignMovingAvg/584856*
_output_shapes
: *
dtype0*
valueB
 *
?#<2/
-batch_normalization_107/AssignMovingAvg/decay?
6batch_normalization_107/AssignMovingAvg/ReadVariableOpReadVariableOp.batch_normalization_107_assignmovingavg_584856*
_output_shapes	
:?*
dtype028
6batch_normalization_107/AssignMovingAvg/ReadVariableOp?
+batch_normalization_107/AssignMovingAvg/subSub>batch_normalization_107/AssignMovingAvg/ReadVariableOp:value:00batch_normalization_107/moments/Squeeze:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*A
_class7
53loc:@batch_normalization_107/AssignMovingAvg/584856*
_output_shapes	
:?2-
+batch_normalization_107/AssignMovingAvg/sub?
+batch_normalization_107/AssignMovingAvg/mulMul/batch_normalization_107/AssignMovingAvg/sub:z:06batch_normalization_107/AssignMovingAvg/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*A
_class7
53loc:@batch_normalization_107/AssignMovingAvg/584856*
_output_shapes	
:?2-
+batch_normalization_107/AssignMovingAvg/mul?
;batch_normalization_107/AssignMovingAvg/AssignSubVariableOpAssignSubVariableOp.batch_normalization_107_assignmovingavg_584856/batch_normalization_107/AssignMovingAvg/mul:z:07^batch_normalization_107/AssignMovingAvg/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*A
_class7
53loc:@batch_normalization_107/AssignMovingAvg/584856*
_output_shapes
 *
dtype02=
;batch_normalization_107/AssignMovingAvg/AssignSubVariableOp?
/batch_normalization_107/AssignMovingAvg_1/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*C
_class9
75loc:@batch_normalization_107/AssignMovingAvg_1/584862*
_output_shapes
: *
dtype0*
valueB
 *
?#<21
/batch_normalization_107/AssignMovingAvg_1/decay?
8batch_normalization_107/AssignMovingAvg_1/ReadVariableOpReadVariableOp0batch_normalization_107_assignmovingavg_1_584862*
_output_shapes	
:?*
dtype02:
8batch_normalization_107/AssignMovingAvg_1/ReadVariableOp?
-batch_normalization_107/AssignMovingAvg_1/subSub@batch_normalization_107/AssignMovingAvg_1/ReadVariableOp:value:02batch_normalization_107/moments/Squeeze_1:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*C
_class9
75loc:@batch_normalization_107/AssignMovingAvg_1/584862*
_output_shapes	
:?2/
-batch_normalization_107/AssignMovingAvg_1/sub?
-batch_normalization_107/AssignMovingAvg_1/mulMul1batch_normalization_107/AssignMovingAvg_1/sub:z:08batch_normalization_107/AssignMovingAvg_1/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*C
_class9
75loc:@batch_normalization_107/AssignMovingAvg_1/584862*
_output_shapes	
:?2/
-batch_normalization_107/AssignMovingAvg_1/mul?
=batch_normalization_107/AssignMovingAvg_1/AssignSubVariableOpAssignSubVariableOp0batch_normalization_107_assignmovingavg_1_5848621batch_normalization_107/AssignMovingAvg_1/mul:z:09^batch_normalization_107/AssignMovingAvg_1/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*C
_class9
75loc:@batch_normalization_107/AssignMovingAvg_1/584862*
_output_shapes
 *
dtype02?
=batch_normalization_107/AssignMovingAvg_1/AssignSubVariableOp?
'batch_normalization_107/batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:2)
'batch_normalization_107/batchnorm/add/y?
%batch_normalization_107/batchnorm/addAddV22batch_normalization_107/moments/Squeeze_1:output:00batch_normalization_107/batchnorm/add/y:output:0*
T0*
_output_shapes	
:?2'
%batch_normalization_107/batchnorm/add?
'batch_normalization_107/batchnorm/RsqrtRsqrt)batch_normalization_107/batchnorm/add:z:0*
T0*
_output_shapes	
:?2)
'batch_normalization_107/batchnorm/Rsqrt?
4batch_normalization_107/batchnorm/mul/ReadVariableOpReadVariableOp=batch_normalization_107_batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype026
4batch_normalization_107/batchnorm/mul/ReadVariableOp?
%batch_normalization_107/batchnorm/mulMul+batch_normalization_107/batchnorm/Rsqrt:y:0<batch_normalization_107/batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?2'
%batch_normalization_107/batchnorm/mul?
'batch_normalization_107/batchnorm/mul_1Mul!activation_124/Relu:activations:0)batch_normalization_107/batchnorm/mul:z:0*
T0*(
_output_shapes
:??????????2)
'batch_normalization_107/batchnorm/mul_1?
'batch_normalization_107/batchnorm/mul_2Mul0batch_normalization_107/moments/Squeeze:output:0)batch_normalization_107/batchnorm/mul:z:0*
T0*
_output_shapes	
:?2)
'batch_normalization_107/batchnorm/mul_2?
0batch_normalization_107/batchnorm/ReadVariableOpReadVariableOp9batch_normalization_107_batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype022
0batch_normalization_107/batchnorm/ReadVariableOp?
%batch_normalization_107/batchnorm/subSub8batch_normalization_107/batchnorm/ReadVariableOp:value:0+batch_normalization_107/batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?2'
%batch_normalization_107/batchnorm/sub?
'batch_normalization_107/batchnorm/add_1AddV2+batch_normalization_107/batchnorm/mul_1:z:0)batch_normalization_107/batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????2)
'batch_normalization_107/batchnorm/add_1y
dropout_53/dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *UU??2
dropout_53/dropout/Const?
dropout_53/dropout/MulMul+batch_normalization_107/batchnorm/add_1:z:0!dropout_53/dropout/Const:output:0*
T0*(
_output_shapes
:??????????2
dropout_53/dropout/Mul?
dropout_53/dropout/ShapeShape+batch_normalization_107/batchnorm/add_1:z:0*
T0*
_output_shapes
:2
dropout_53/dropout/Shape?
/dropout_53/dropout/random_uniform/RandomUniformRandomUniform!dropout_53/dropout/Shape:output:0*
T0*(
_output_shapes
:??????????*
dtype021
/dropout_53/dropout/random_uniform/RandomUniform?
!dropout_53/dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *???>2#
!dropout_53/dropout/GreaterEqual/y?
dropout_53/dropout/GreaterEqualGreaterEqual8dropout_53/dropout/random_uniform/RandomUniform:output:0*dropout_53/dropout/GreaterEqual/y:output:0*
T0*(
_output_shapes
:??????????2!
dropout_53/dropout/GreaterEqual?
dropout_53/dropout/CastCast#dropout_53/dropout/GreaterEqual:z:0*

DstT0*

SrcT0
*(
_output_shapes
:??????????2
dropout_53/dropout/Cast?
dropout_53/dropout/Mul_1Muldropout_53/dropout/Mul:z:0dropout_53/dropout/Cast:y:0*
T0*(
_output_shapes
:??????????2
dropout_53/dropout/Mul_1?
dense_53/MatMul/ReadVariableOpReadVariableOp'dense_53_matmul_readvariableop_resource*
_output_shapes
:	?*
dtype02 
dense_53/MatMul/ReadVariableOp?
dense_53/MatMulMatMuldropout_53/dropout/Mul_1:z:0&dense_53/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
dense_53/MatMul?
dense_53/BiasAdd/ReadVariableOpReadVariableOp(dense_53_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02!
dense_53/BiasAdd/ReadVariableOp?
dense_53/BiasAddBiasAdddense_53/MatMul:product:0'dense_53/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
dense_53/BiasAdd?
activation_125/SoftmaxSoftmaxdense_53/BiasAdd:output:0*
T0*'
_output_shapes
:?????????2
activation_125/Softmax?
IdentityIdentity activation_125/Softmax:softmax:0'^batch_normalization_102/AssignNewValue)^batch_normalization_102/AssignNewValue_18^batch_normalization_102/FusedBatchNormV3/ReadVariableOp:^batch_normalization_102/FusedBatchNormV3/ReadVariableOp_1'^batch_normalization_102/ReadVariableOp)^batch_normalization_102/ReadVariableOp_1'^batch_normalization_103/AssignNewValue)^batch_normalization_103/AssignNewValue_18^batch_normalization_103/FusedBatchNormV3/ReadVariableOp:^batch_normalization_103/FusedBatchNormV3/ReadVariableOp_1'^batch_normalization_103/ReadVariableOp)^batch_normalization_103/ReadVariableOp_1'^batch_normalization_104/AssignNewValue)^batch_normalization_104/AssignNewValue_18^batch_normalization_104/FusedBatchNormV3/ReadVariableOp:^batch_normalization_104/FusedBatchNormV3/ReadVariableOp_1'^batch_normalization_104/ReadVariableOp)^batch_normalization_104/ReadVariableOp_1'^batch_normalization_105/AssignNewValue)^batch_normalization_105/AssignNewValue_18^batch_normalization_105/FusedBatchNormV3/ReadVariableOp:^batch_normalization_105/FusedBatchNormV3/ReadVariableOp_1'^batch_normalization_105/ReadVariableOp)^batch_normalization_105/ReadVariableOp_1<^batch_normalization_106/AssignMovingAvg/AssignSubVariableOp7^batch_normalization_106/AssignMovingAvg/ReadVariableOp>^batch_normalization_106/AssignMovingAvg_1/AssignSubVariableOp9^batch_normalization_106/AssignMovingAvg_1/ReadVariableOp1^batch_normalization_106/batchnorm/ReadVariableOp5^batch_normalization_106/batchnorm/mul/ReadVariableOp<^batch_normalization_107/AssignMovingAvg/AssignSubVariableOp7^batch_normalization_107/AssignMovingAvg/ReadVariableOp>^batch_normalization_107/AssignMovingAvg_1/AssignSubVariableOp9^batch_normalization_107/AssignMovingAvg_1/ReadVariableOp1^batch_normalization_107/batchnorm/ReadVariableOp5^batch_normalization_107/batchnorm/mul/ReadVariableOp!^conv2d_68/BiasAdd/ReadVariableOp ^conv2d_68/Conv2D/ReadVariableOp!^conv2d_69/BiasAdd/ReadVariableOp ^conv2d_69/Conv2D/ReadVariableOp!^conv2d_70/BiasAdd/ReadVariableOp ^conv2d_70/Conv2D/ReadVariableOp!^conv2d_71/BiasAdd/ReadVariableOp ^conv2d_71/Conv2D/ReadVariableOp ^dense_51/BiasAdd/ReadVariableOp^dense_51/MatMul/ReadVariableOp ^dense_52/BiasAdd/ReadVariableOp^dense_52/MatMul/ReadVariableOp ^dense_53/BiasAdd/ReadVariableOp^dense_53/MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::2P
&batch_normalization_102/AssignNewValue&batch_normalization_102/AssignNewValue2T
(batch_normalization_102/AssignNewValue_1(batch_normalization_102/AssignNewValue_12r
7batch_normalization_102/FusedBatchNormV3/ReadVariableOp7batch_normalization_102/FusedBatchNormV3/ReadVariableOp2v
9batch_normalization_102/FusedBatchNormV3/ReadVariableOp_19batch_normalization_102/FusedBatchNormV3/ReadVariableOp_12P
&batch_normalization_102/ReadVariableOp&batch_normalization_102/ReadVariableOp2T
(batch_normalization_102/ReadVariableOp_1(batch_normalization_102/ReadVariableOp_12P
&batch_normalization_103/AssignNewValue&batch_normalization_103/AssignNewValue2T
(batch_normalization_103/AssignNewValue_1(batch_normalization_103/AssignNewValue_12r
7batch_normalization_103/FusedBatchNormV3/ReadVariableOp7batch_normalization_103/FusedBatchNormV3/ReadVariableOp2v
9batch_normalization_103/FusedBatchNormV3/ReadVariableOp_19batch_normalization_103/FusedBatchNormV3/ReadVariableOp_12P
&batch_normalization_103/ReadVariableOp&batch_normalization_103/ReadVariableOp2T
(batch_normalization_103/ReadVariableOp_1(batch_normalization_103/ReadVariableOp_12P
&batch_normalization_104/AssignNewValue&batch_normalization_104/AssignNewValue2T
(batch_normalization_104/AssignNewValue_1(batch_normalization_104/AssignNewValue_12r
7batch_normalization_104/FusedBatchNormV3/ReadVariableOp7batch_normalization_104/FusedBatchNormV3/ReadVariableOp2v
9batch_normalization_104/FusedBatchNormV3/ReadVariableOp_19batch_normalization_104/FusedBatchNormV3/ReadVariableOp_12P
&batch_normalization_104/ReadVariableOp&batch_normalization_104/ReadVariableOp2T
(batch_normalization_104/ReadVariableOp_1(batch_normalization_104/ReadVariableOp_12P
&batch_normalization_105/AssignNewValue&batch_normalization_105/AssignNewValue2T
(batch_normalization_105/AssignNewValue_1(batch_normalization_105/AssignNewValue_12r
7batch_normalization_105/FusedBatchNormV3/ReadVariableOp7batch_normalization_105/FusedBatchNormV3/ReadVariableOp2v
9batch_normalization_105/FusedBatchNormV3/ReadVariableOp_19batch_normalization_105/FusedBatchNormV3/ReadVariableOp_12P
&batch_normalization_105/ReadVariableOp&batch_normalization_105/ReadVariableOp2T
(batch_normalization_105/ReadVariableOp_1(batch_normalization_105/ReadVariableOp_12z
;batch_normalization_106/AssignMovingAvg/AssignSubVariableOp;batch_normalization_106/AssignMovingAvg/AssignSubVariableOp2p
6batch_normalization_106/AssignMovingAvg/ReadVariableOp6batch_normalization_106/AssignMovingAvg/ReadVariableOp2~
=batch_normalization_106/AssignMovingAvg_1/AssignSubVariableOp=batch_normalization_106/AssignMovingAvg_1/AssignSubVariableOp2t
8batch_normalization_106/AssignMovingAvg_1/ReadVariableOp8batch_normalization_106/AssignMovingAvg_1/ReadVariableOp2d
0batch_normalization_106/batchnorm/ReadVariableOp0batch_normalization_106/batchnorm/ReadVariableOp2l
4batch_normalization_106/batchnorm/mul/ReadVariableOp4batch_normalization_106/batchnorm/mul/ReadVariableOp2z
;batch_normalization_107/AssignMovingAvg/AssignSubVariableOp;batch_normalization_107/AssignMovingAvg/AssignSubVariableOp2p
6batch_normalization_107/AssignMovingAvg/ReadVariableOp6batch_normalization_107/AssignMovingAvg/ReadVariableOp2~
=batch_normalization_107/AssignMovingAvg_1/AssignSubVariableOp=batch_normalization_107/AssignMovingAvg_1/AssignSubVariableOp2t
8batch_normalization_107/AssignMovingAvg_1/ReadVariableOp8batch_normalization_107/AssignMovingAvg_1/ReadVariableOp2d
0batch_normalization_107/batchnorm/ReadVariableOp0batch_normalization_107/batchnorm/ReadVariableOp2l
4batch_normalization_107/batchnorm/mul/ReadVariableOp4batch_normalization_107/batchnorm/mul/ReadVariableOp2D
 conv2d_68/BiasAdd/ReadVariableOp conv2d_68/BiasAdd/ReadVariableOp2B
conv2d_68/Conv2D/ReadVariableOpconv2d_68/Conv2D/ReadVariableOp2D
 conv2d_69/BiasAdd/ReadVariableOp conv2d_69/BiasAdd/ReadVariableOp2B
conv2d_69/Conv2D/ReadVariableOpconv2d_69/Conv2D/ReadVariableOp2D
 conv2d_70/BiasAdd/ReadVariableOp conv2d_70/BiasAdd/ReadVariableOp2B
conv2d_70/Conv2D/ReadVariableOpconv2d_70/Conv2D/ReadVariableOp2D
 conv2d_71/BiasAdd/ReadVariableOp conv2d_71/BiasAdd/ReadVariableOp2B
conv2d_71/Conv2D/ReadVariableOpconv2d_71/Conv2D/ReadVariableOp2B
dense_51/BiasAdd/ReadVariableOpdense_51/BiasAdd/ReadVariableOp2@
dense_51/MatMul/ReadVariableOpdense_51/MatMul/ReadVariableOp2B
dense_52/BiasAdd/ReadVariableOpdense_52/BiasAdd/ReadVariableOp2@
dense_52/MatMul/ReadVariableOpdense_52/MatMul/ReadVariableOp2B
dense_53/BiasAdd/ReadVariableOpdense_53/BiasAdd/ReadVariableOp2@
dense_53/MatMul/ReadVariableOpdense_53/MatMul/ReadVariableOp:W S
/
_output_shapes
:?????????@@
 
_user_specified_nameinputs
?
K
/__inference_activation_125_layer_call_fn_586190

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_125_layer_call_and_return_conditional_losses_5841012
PartitionedCalll
IdentityIdentityPartitionedCall:output:0*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*&
_input_shapes
:?????????:O K
'
_output_shapes
:?????????
 
_user_specified_nameinputs
?
f
J__inference_activation_123_layer_call_and_return_conditional_losses_585898

inputs
identityO
ReluReluinputs*
T0*(
_output_shapes
:??????????2
Relug
IdentityIdentityRelu:activations:0*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
d
F__inference_dropout_52_layer_call_and_return_conditional_losses_586002

inputs

identity_1[
IdentityIdentityinputs*
T0*(
_output_shapes
:??????????2

Identityj

Identity_1IdentityIdentity:output:0*
T0*(
_output_shapes
:??????????2

Identity_1"!

identity_1Identity_1:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
d
F__inference_dropout_51_layer_call_and_return_conditional_losses_583821

inputs

identity_1b
IdentityIdentityinputs*
T0*/
_output_shapes
:?????????2

Identityq

Identity_1IdentityIdentity:output:0*
T0*/
_output_shapes
:?????????2

Identity_1"!

identity_1Identity_1:output:0*.
_input_shapes
:?????????:W S
/
_output_shapes
:?????????
 
_user_specified_nameinputs
?
K
/__inference_activation_123_layer_call_fn_585903

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_123_layer_call_and_return_conditional_losses_5838792
PartitionedCallm
IdentityIdentityPartitionedCall:output:0*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*'
_input_shapes
:??????????:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
?
8__inference_batch_normalization_106_layer_call_fn_585985

inputs
unknown
	unknown_0
	unknown_1
	unknown_2
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2*
Tin	
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_106_layer_call_and_return_conditional_losses_5831992
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*7
_input_shapes&
$:??????????::::22
StatefulPartitionedCallStatefulPartitionedCall:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?
?
I__inference_sequential_17_layer_call_and_return_conditional_losses_584217
conv2d_68_input
conv2d_68_584113
conv2d_68_584115"
batch_normalization_102_584119"
batch_normalization_102_584121"
batch_normalization_102_584123"
batch_normalization_102_584125
conv2d_69_584128
conv2d_69_584130"
batch_normalization_103_584134"
batch_normalization_103_584136"
batch_normalization_103_584138"
batch_normalization_103_584140
conv2d_70_584144
conv2d_70_584146"
batch_normalization_104_584150"
batch_normalization_104_584152"
batch_normalization_104_584154"
batch_normalization_104_584156
conv2d_71_584159
conv2d_71_584161"
batch_normalization_105_584165"
batch_normalization_105_584167"
batch_normalization_105_584169"
batch_normalization_105_584171
dense_51_584177
dense_51_584179"
batch_normalization_106_584183"
batch_normalization_106_584185"
batch_normalization_106_584187"
batch_normalization_106_584189
dense_52_584194
dense_52_584196"
batch_normalization_107_584200"
batch_normalization_107_584202"
batch_normalization_107_584204"
batch_normalization_107_584206
dense_53_584210
dense_53_584212
identity??/batch_normalization_102/StatefulPartitionedCall?/batch_normalization_103/StatefulPartitionedCall?/batch_normalization_104/StatefulPartitionedCall?/batch_normalization_105/StatefulPartitionedCall?/batch_normalization_106/StatefulPartitionedCall?/batch_normalization_107/StatefulPartitionedCall?!conv2d_68/StatefulPartitionedCall?!conv2d_69/StatefulPartitionedCall?!conv2d_70/StatefulPartitionedCall?!conv2d_71/StatefulPartitionedCall? dense_51/StatefulPartitionedCall? dense_52/StatefulPartitionedCall? dense_53/StatefulPartitionedCall?
!conv2d_68/StatefulPartitionedCallStatefulPartitionedCallconv2d_68_inputconv2d_68_584113conv2d_68_584115*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<<<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_68_layer_call_and_return_conditional_losses_5833642#
!conv2d_68/StatefulPartitionedCall?
activation_119/PartitionedCallPartitionedCall*conv2d_68/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<<<* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_119_layer_call_and_return_conditional_losses_5833852 
activation_119/PartitionedCall?
/batch_normalization_102/StatefulPartitionedCallStatefulPartitionedCall'activation_119/PartitionedCall:output:0batch_normalization_102_584119batch_normalization_102_584121batch_normalization_102_584123batch_normalization_102_584125*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<<<*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_58343021
/batch_normalization_102/StatefulPartitionedCall?
!conv2d_69/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_102/StatefulPartitionedCall:output:0conv2d_69_584128conv2d_69_584130*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????88<*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_69_layer_call_and_return_conditional_losses_5834762#
!conv2d_69/StatefulPartitionedCall?
activation_120/PartitionedCallPartitionedCall*conv2d_69/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????88<* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_120_layer_call_and_return_conditional_losses_5834972 
activation_120/PartitionedCall?
/batch_normalization_103/StatefulPartitionedCallStatefulPartitionedCall'activation_120/PartitionedCall:output:0batch_normalization_103_584134batch_normalization_103_584136batch_normalization_103_584138batch_normalization_103_584140*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????88<*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_58354221
/batch_normalization_103/StatefulPartitionedCall?
 max_pooling2d_34/PartitionedCallPartitionedCall8batch_normalization_103/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????<* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *U
fPRN
L__inference_max_pooling2d_34_layer_call_and_return_conditional_losses_5828442"
 max_pooling2d_34/PartitionedCall?
!conv2d_70/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_34/PartitionedCall:output:0conv2d_70_584144conv2d_70_584146*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_70_layer_call_and_return_conditional_losses_5835892#
!conv2d_70/StatefulPartitionedCall?
activation_121/PartitionedCallPartitionedCall*conv2d_70/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_121_layer_call_and_return_conditional_losses_5836102 
activation_121/PartitionedCall?
/batch_normalization_104/StatefulPartitionedCallStatefulPartitionedCall'activation_121/PartitionedCall:output:0batch_normalization_104_584150batch_normalization_104_584152batch_normalization_104_584154batch_normalization_104_584156*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_58365521
/batch_normalization_104/StatefulPartitionedCall?
!conv2d_71/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_104/StatefulPartitionedCall:output:0conv2d_71_584159conv2d_71_584161*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *N
fIRG
E__inference_conv2d_71_layer_call_and_return_conditional_losses_5837012#
!conv2d_71/StatefulPartitionedCall?
activation_122/PartitionedCallPartitionedCall*conv2d_71/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_122_layer_call_and_return_conditional_losses_5837222 
activation_122/PartitionedCall?
/batch_normalization_105/StatefulPartitionedCallStatefulPartitionedCall'activation_122/PartitionedCall:output:0batch_normalization_105_584165batch_normalization_105_584167batch_normalization_105_584169batch_normalization_105_584171*
Tin	
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_58376721
/batch_normalization_105/StatefulPartitionedCall?
 max_pooling2d_35/PartitionedCallPartitionedCall8batch_normalization_105/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *U
fPRN
L__inference_max_pooling2d_35_layer_call_and_return_conditional_losses_5830642"
 max_pooling2d_35/PartitionedCall?
dropout_51/PartitionedCallPartitionedCall)max_pooling2d_35/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 */
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_51_layer_call_and_return_conditional_losses_5838212
dropout_51/PartitionedCall?
flatten_34/PartitionedCallPartitionedCall#dropout_51/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????!* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_flatten_34_layer_call_and_return_conditional_losses_5838402
flatten_34/PartitionedCall?
 dense_51/StatefulPartitionedCallStatefulPartitionedCall#flatten_34/PartitionedCall:output:0dense_51_584177dense_51_584179*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *M
fHRF
D__inference_dense_51_layer_call_and_return_conditional_losses_5838582"
 dense_51/StatefulPartitionedCall?
activation_123/PartitionedCallPartitionedCall)dense_51/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_123_layer_call_and_return_conditional_losses_5838792 
activation_123/PartitionedCall?
/batch_normalization_106/StatefulPartitionedCallStatefulPartitionedCall'activation_123/PartitionedCall:output:0batch_normalization_106_584183batch_normalization_106_584185batch_normalization_106_584187batch_normalization_106_584189*
Tin	
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_106_layer_call_and_return_conditional_losses_58319921
/batch_normalization_106/StatefulPartitionedCall?
dropout_52/PartitionedCallPartitionedCall8batch_normalization_106/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_52_layer_call_and_return_conditional_losses_5839392
dropout_52/PartitionedCall?
flatten_35/PartitionedCallPartitionedCall#dropout_52/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_flatten_35_layer_call_and_return_conditional_losses_5839582
flatten_35/PartitionedCall?
 dense_52/StatefulPartitionedCallStatefulPartitionedCall#flatten_35/PartitionedCall:output:0dense_52_584194dense_52_584196*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *M
fHRF
D__inference_dense_52_layer_call_and_return_conditional_losses_5839762"
 dense_52/StatefulPartitionedCall?
activation_124/PartitionedCallPartitionedCall)dense_52/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_124_layer_call_and_return_conditional_losses_5839972 
activation_124/PartitionedCall?
/batch_normalization_107/StatefulPartitionedCallStatefulPartitionedCall'activation_124/PartitionedCall:output:0batch_normalization_107_584200batch_normalization_107_584202batch_normalization_107_584204batch_normalization_107_584206*
Tin	
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????*&
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *\
fWRU
S__inference_batch_normalization_107_layer_call_and_return_conditional_losses_58333921
/batch_normalization_107/StatefulPartitionedCall?
dropout_53/PartitionedCallPartitionedCall8batch_normalization_107/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *(
_output_shapes
:??????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *O
fJRH
F__inference_dropout_53_layer_call_and_return_conditional_losses_5840572
dropout_53/PartitionedCall?
 dense_53/StatefulPartitionedCallStatefulPartitionedCall#dropout_53/PartitionedCall:output:0dense_53_584210dense_53_584212*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*$
_read_only_resource_inputs
*0
config_proto 

CPU

GPU2*0J 8? *M
fHRF
D__inference_dense_53_layer_call_and_return_conditional_losses_5840802"
 dense_53/StatefulPartitionedCall?
activation_125/PartitionedCallPartitionedCall)dense_53/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *0
config_proto 

CPU

GPU2*0J 8? *S
fNRL
J__inference_activation_125_layer_call_and_return_conditional_losses_5841012 
activation_125/PartitionedCall?
IdentityIdentity'activation_125/PartitionedCall:output:00^batch_normalization_102/StatefulPartitionedCall0^batch_normalization_103/StatefulPartitionedCall0^batch_normalization_104/StatefulPartitionedCall0^batch_normalization_105/StatefulPartitionedCall0^batch_normalization_106/StatefulPartitionedCall0^batch_normalization_107/StatefulPartitionedCall"^conv2d_68/StatefulPartitionedCall"^conv2d_69/StatefulPartitionedCall"^conv2d_70/StatefulPartitionedCall"^conv2d_71/StatefulPartitionedCall!^dense_51/StatefulPartitionedCall!^dense_52/StatefulPartitionedCall!^dense_53/StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::2b
/batch_normalization_102/StatefulPartitionedCall/batch_normalization_102/StatefulPartitionedCall2b
/batch_normalization_103/StatefulPartitionedCall/batch_normalization_103/StatefulPartitionedCall2b
/batch_normalization_104/StatefulPartitionedCall/batch_normalization_104/StatefulPartitionedCall2b
/batch_normalization_105/StatefulPartitionedCall/batch_normalization_105/StatefulPartitionedCall2b
/batch_normalization_106/StatefulPartitionedCall/batch_normalization_106/StatefulPartitionedCall2b
/batch_normalization_107/StatefulPartitionedCall/batch_normalization_107/StatefulPartitionedCall2F
!conv2d_68/StatefulPartitionedCall!conv2d_68/StatefulPartitionedCall2F
!conv2d_69/StatefulPartitionedCall!conv2d_69/StatefulPartitionedCall2F
!conv2d_70/StatefulPartitionedCall!conv2d_70/StatefulPartitionedCall2F
!conv2d_71/StatefulPartitionedCall!conv2d_71/StatefulPartitionedCall2D
 dense_51/StatefulPartitionedCall dense_51/StatefulPartitionedCall2D
 dense_52/StatefulPartitionedCall dense_52/StatefulPartitionedCall2D
 dense_53/StatefulPartitionedCall dense_53/StatefulPartitionedCall:` \
/
_output_shapes
:?????????@@
)
_user_specified_nameconv2d_68_input
?
f
J__inference_activation_119_layer_call_and_return_conditional_losses_585232

inputs
identityV
ReluReluinputs*
T0*/
_output_shapes
:?????????<<<2
Relun
IdentityIdentityRelu:activations:0*
T0*/
_output_shapes
:?????????<<<2

Identity"
identityIdentity:output:0*.
_input_shapes
:?????????<<<:W S
/
_output_shapes
:?????????<<<
 
_user_specified_nameinputs
?	
?
D__inference_dense_53_layer_call_and_return_conditional_losses_586171

inputs"
matmul_readvariableop_resource#
biasadd_readvariableop_resource
identity??BiasAdd/ReadVariableOp?MatMul/ReadVariableOp?
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes
:	?*
dtype02
MatMul/ReadVariableOps
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
MatMul?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2	
BiasAdd?
IdentityIdentityBiasAdd:output:0^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*/
_input_shapes
:??????????::20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs
?0
?
S__inference_batch_normalization_106_layer_call_and_return_conditional_losses_583166

inputs
assignmovingavg_583141
assignmovingavg_1_583147)
%batchnorm_mul_readvariableop_resource%
!batchnorm_readvariableop_resource
identity??#AssignMovingAvg/AssignSubVariableOp?AssignMovingAvg/ReadVariableOp?%AssignMovingAvg_1/AssignSubVariableOp? AssignMovingAvg_1/ReadVariableOp?batchnorm/ReadVariableOp?batchnorm/mul/ReadVariableOp?
moments/mean/reduction_indicesConst*
_output_shapes
:*
dtype0*
valueB: 2 
moments/mean/reduction_indices?
moments/meanMeaninputs'moments/mean/reduction_indices:output:0*
T0*
_output_shapes
:	?*
	keep_dims(2
moments/mean}
moments/StopGradientStopGradientmoments/mean:output:0*
T0*
_output_shapes
:	?2
moments/StopGradient?
moments/SquaredDifferenceSquaredDifferenceinputsmoments/StopGradient:output:0*
T0*(
_output_shapes
:??????????2
moments/SquaredDifference?
"moments/variance/reduction_indicesConst*
_output_shapes
:*
dtype0*
valueB: 2$
"moments/variance/reduction_indices?
moments/varianceMeanmoments/SquaredDifference:z:0+moments/variance/reduction_indices:output:0*
T0*
_output_shapes
:	?*
	keep_dims(2
moments/variance?
moments/SqueezeSqueezemoments/mean:output:0*
T0*
_output_shapes	
:?*
squeeze_dims
 2
moments/Squeeze?
moments/Squeeze_1Squeezemoments/variance:output:0*
T0*
_output_shapes	
:?*
squeeze_dims
 2
moments/Squeeze_1?
AssignMovingAvg/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*)
_class
loc:@AssignMovingAvg/583141*
_output_shapes
: *
dtype0*
valueB
 *
?#<2
AssignMovingAvg/decay?
AssignMovingAvg/ReadVariableOpReadVariableOpassignmovingavg_583141*
_output_shapes	
:?*
dtype02 
AssignMovingAvg/ReadVariableOp?
AssignMovingAvg/subSub&AssignMovingAvg/ReadVariableOp:value:0moments/Squeeze:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*)
_class
loc:@AssignMovingAvg/583141*
_output_shapes	
:?2
AssignMovingAvg/sub?
AssignMovingAvg/mulMulAssignMovingAvg/sub:z:0AssignMovingAvg/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*)
_class
loc:@AssignMovingAvg/583141*
_output_shapes	
:?2
AssignMovingAvg/mul?
#AssignMovingAvg/AssignSubVariableOpAssignSubVariableOpassignmovingavg_583141AssignMovingAvg/mul:z:0^AssignMovingAvg/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*)
_class
loc:@AssignMovingAvg/583141*
_output_shapes
 *
dtype02%
#AssignMovingAvg/AssignSubVariableOp?
AssignMovingAvg_1/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*+
_class!
loc:@AssignMovingAvg_1/583147*
_output_shapes
: *
dtype0*
valueB
 *
?#<2
AssignMovingAvg_1/decay?
 AssignMovingAvg_1/ReadVariableOpReadVariableOpassignmovingavg_1_583147*
_output_shapes	
:?*
dtype02"
 AssignMovingAvg_1/ReadVariableOp?
AssignMovingAvg_1/subSub(AssignMovingAvg_1/ReadVariableOp:value:0moments/Squeeze_1:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*+
_class!
loc:@AssignMovingAvg_1/583147*
_output_shapes	
:?2
AssignMovingAvg_1/sub?
AssignMovingAvg_1/mulMulAssignMovingAvg_1/sub:z:0 AssignMovingAvg_1/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*+
_class!
loc:@AssignMovingAvg_1/583147*
_output_shapes	
:?2
AssignMovingAvg_1/mul?
%AssignMovingAvg_1/AssignSubVariableOpAssignSubVariableOpassignmovingavg_1_583147AssignMovingAvg_1/mul:z:0!^AssignMovingAvg_1/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*+
_class!
loc:@AssignMovingAvg_1/583147*
_output_shapes
 *
dtype02'
%AssignMovingAvg_1/AssignSubVariableOpg
batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:2
batchnorm/add/y?
batchnorm/addAddV2moments/Squeeze_1:output:0batchnorm/add/y:output:0*
T0*
_output_shapes	
:?2
batchnorm/addd
batchnorm/RsqrtRsqrtbatchnorm/add:z:0*
T0*
_output_shapes	
:?2
batchnorm/Rsqrt?
batchnorm/mul/ReadVariableOpReadVariableOp%batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype02
batchnorm/mul/ReadVariableOp?
batchnorm/mulMulbatchnorm/Rsqrt:y:0$batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?2
batchnorm/mulw
batchnorm/mul_1Mulinputsbatchnorm/mul:z:0*
T0*(
_output_shapes
:??????????2
batchnorm/mul_1|
batchnorm/mul_2Mulmoments/Squeeze:output:0batchnorm/mul:z:0*
T0*
_output_shapes	
:?2
batchnorm/mul_2?
batchnorm/ReadVariableOpReadVariableOp!batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype02
batchnorm/ReadVariableOp?
batchnorm/subSub batchnorm/ReadVariableOp:value:0batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?2
batchnorm/sub?
batchnorm/add_1AddV2batchnorm/mul_1:z:0batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????2
batchnorm/add_1?
IdentityIdentitybatchnorm/add_1:z:0$^AssignMovingAvg/AssignSubVariableOp^AssignMovingAvg/ReadVariableOp&^AssignMovingAvg_1/AssignSubVariableOp!^AssignMovingAvg_1/ReadVariableOp^batchnorm/ReadVariableOp^batchnorm/mul/ReadVariableOp*
T0*(
_output_shapes
:??????????2

Identity"
identityIdentity:output:0*7
_input_shapes&
$:??????????::::2J
#AssignMovingAvg/AssignSubVariableOp#AssignMovingAvg/AssignSubVariableOp2@
AssignMovingAvg/ReadVariableOpAssignMovingAvg/ReadVariableOp2N
%AssignMovingAvg_1/AssignSubVariableOp%AssignMovingAvg_1/AssignSubVariableOp2D
 AssignMovingAvg_1/ReadVariableOp AssignMovingAvg_1/ReadVariableOp24
batchnorm/ReadVariableOpbatchnorm/ReadVariableOp2<
batchnorm/mul/ReadVariableOpbatchnorm/mul/ReadVariableOp:P L
(
_output_shapes
:??????????
 
_user_specified_nameinputs"?L
saver_filename:0StatefulPartitionedCall_1:0StatefulPartitionedCall_28"
saved_model_main_op

NoOp*>
__saved_model_init_op%#
__saved_model_init_op

NoOp*?
serving_default?
S
conv2d_68_input@
!serving_default_conv2d_68_input:0?????????@@B
activation_1250
StatefulPartitionedCall:0?????????tensorflow/serving/predict:??
??
layer_with_weights-0
layer-0
layer-1
layer_with_weights-1
layer-2
layer_with_weights-2
layer-3
layer-4
layer_with_weights-3
layer-5
layer-6
layer_with_weights-4
layer-7
	layer-8

layer_with_weights-5

layer-9
layer_with_weights-6
layer-10
layer-11
layer_with_weights-7
layer-12
layer-13
layer-14
layer-15
layer_with_weights-8
layer-16
layer-17
layer_with_weights-9
layer-18
layer-19
layer-20
layer_with_weights-10
layer-21
layer-22
layer_with_weights-11
layer-23
layer-24
layer_with_weights-12
layer-25
layer-26
	optimizer
	variables
regularization_losses
trainable_variables
 	keras_api
!
signatures
?__call__
+?&call_and_return_all_conditional_losses
?_default_save_signature"??
_tf_keras_sequential??{"class_name": "Sequential", "name": "sequential_17", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "must_restore_from_config": false, "config": {"name": "sequential_17", "layers": [{"class_name": "InputLayer", "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 64, 64, 3]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "conv2d_68_input"}}, {"class_name": "Conv2D", "config": {"name": "conv2d_68", "trainable": true, "batch_input_shape": {"class_name": "__tuple__", "items": [null, 64, 64, 3]}, "dtype": "float32", "filters": 60, "kernel_size": {"class_name": "__tuple__", "items": [5, 5]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_119", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_102", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "Conv2D", "config": {"name": "conv2d_69", "trainable": true, "dtype": "float32", "filters": 60, "kernel_size": {"class_name": "__tuple__", "items": [5, 5]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_120", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_103", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "MaxPooling2D", "config": {"name": "max_pooling2d_34", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}}, {"class_name": "Conv2D", "config": {"name": "conv2d_70", "trainable": true, "dtype": "float32", "filters": 30, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_121", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_104", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "Conv2D", "config": {"name": "conv2d_71", "trainable": true, "dtype": "float32", "filters": 30, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_122", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_105", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "MaxPooling2D", "config": {"name": "max_pooling2d_35", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}}, {"class_name": "Dropout", "config": {"name": "dropout_51", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}, {"class_name": "Flatten", "config": {"name": "flatten_34", "trainable": true, "dtype": "float32", "data_format": "channels_last"}}, {"class_name": "Dense", "config": {"name": "dense_51", "trainable": true, "dtype": "float32", "units": 300, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_123", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_106", "trainable": true, "dtype": "float32", "axis": [1], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "Dropout", "config": {"name": "dropout_52", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}, {"class_name": "Flatten", "config": {"name": "flatten_35", "trainable": true, "dtype": "float32", "data_format": "channels_last"}}, {"class_name": "Dense", "config": {"name": "dense_52", "trainable": true, "dtype": "float32", "units": 300, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_124", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_107", "trainable": true, "dtype": "float32", "axis": [1], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "Dropout", "config": {"name": "dropout_53", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}, {"class_name": "Dense", "config": {"name": "dense_53", "trainable": true, "dtype": "float32", "units": 6, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_125", "trainable": true, "dtype": "float32", "activation": "softmax"}}]}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 4, "axes": {"-1": 3}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 64, 64, 3]}, "is_graph_network": true, "keras_version": "2.4.0", "backend": "tensorflow", "model_config": {"class_name": "Sequential", "config": {"name": "sequential_17", "layers": [{"class_name": "InputLayer", "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 64, 64, 3]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "conv2d_68_input"}}, {"class_name": "Conv2D", "config": {"name": "conv2d_68", "trainable": true, "batch_input_shape": {"class_name": "__tuple__", "items": [null, 64, 64, 3]}, "dtype": "float32", "filters": 60, "kernel_size": {"class_name": "__tuple__", "items": [5, 5]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_119", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_102", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "Conv2D", "config": {"name": "conv2d_69", "trainable": true, "dtype": "float32", "filters": 60, "kernel_size": {"class_name": "__tuple__", "items": [5, 5]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_120", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_103", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "MaxPooling2D", "config": {"name": "max_pooling2d_34", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}}, {"class_name": "Conv2D", "config": {"name": "conv2d_70", "trainable": true, "dtype": "float32", "filters": 30, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_121", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_104", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "Conv2D", "config": {"name": "conv2d_71", "trainable": true, "dtype": "float32", "filters": 30, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_122", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_105", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "MaxPooling2D", "config": {"name": "max_pooling2d_35", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}}, {"class_name": "Dropout", "config": {"name": "dropout_51", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}, {"class_name": "Flatten", "config": {"name": "flatten_34", "trainable": true, "dtype": "float32", "data_format": "channels_last"}}, {"class_name": "Dense", "config": {"name": "dense_51", "trainable": true, "dtype": "float32", "units": 300, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_123", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_106", "trainable": true, "dtype": "float32", "axis": [1], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "Dropout", "config": {"name": "dropout_52", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}, {"class_name": "Flatten", "config": {"name": "flatten_35", "trainable": true, "dtype": "float32", "data_format": "channels_last"}}, {"class_name": "Dense", "config": {"name": "dense_52", "trainable": true, "dtype": "float32", "units": 300, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_124", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_107", "trainable": true, "dtype": "float32", "axis": [1], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "Dropout", "config": {"name": "dropout_53", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}, {"class_name": "Dense", "config": {"name": "dense_53", "trainable": true, "dtype": "float32", "units": 6, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_125", "trainable": true, "dtype": "float32", "activation": "softmax"}}]}}, "training_config": {"loss": "categorical_crossentropy", "metrics": [[{"class_name": "MeanMetricWrapper", "config": {"name": "accuracy", "dtype": "float32", "fn": "categorical_accuracy"}}]], "weighted_metrics": null, "loss_weights": null, "optimizer_config": {"class_name": "Adam", "config": {"name": "Adam", "learning_rate": 0.0010000000474974513, "decay": 2.6666666599339806e-05, "beta_1": 0.8999999761581421, "beta_2": 0.9990000128746033, "epsilon": 1e-07, "amsgrad": false}}}}
?


"kernel
#bias
$	variables
%regularization_losses
&trainable_variables
'	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?	
_tf_keras_layer?	{"class_name": "Conv2D", "name": "conv2d_68", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": {"class_name": "__tuple__", "items": [null, 64, 64, 3]}, "stateful": false, "must_restore_from_config": false, "config": {"name": "conv2d_68", "trainable": true, "batch_input_shape": {"class_name": "__tuple__", "items": [null, 64, 64, 3]}, "dtype": "float32", "filters": 60, "kernel_size": {"class_name": "__tuple__", "items": [5, 5]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 4, "axes": {"-1": 3}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 64, 64, 3]}}
?
(	variables
)regularization_losses
*trainable_variables
+	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_119", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_119", "trainable": true, "dtype": "float32", "activation": "relu"}}
?	
,axis
	-gamma
.beta
/moving_mean
0moving_variance
1	variables
2regularization_losses
3trainable_variables
4	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "BatchNormalization", "name": "batch_normalization_102", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "batch_normalization_102", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 4, "max_ndim": null, "min_ndim": null, "axes": {"3": 60}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 60, 60, 60]}}
?	

5kernel
6bias
7	variables
8regularization_losses
9trainable_variables
:	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Conv2D", "name": "conv2d_69", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "conv2d_69", "trainable": true, "dtype": "float32", "filters": 60, "kernel_size": {"class_name": "__tuple__", "items": [5, 5]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 4, "axes": {"-1": 60}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 60, 60, 60]}}
?
;	variables
<regularization_losses
=trainable_variables
>	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_120", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_120", "trainable": true, "dtype": "float32", "activation": "relu"}}
?	
?axis
	@gamma
Abeta
Bmoving_mean
Cmoving_variance
D	variables
Eregularization_losses
Ftrainable_variables
G	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "BatchNormalization", "name": "batch_normalization_103", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "batch_normalization_103", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 4, "max_ndim": null, "min_ndim": null, "axes": {"3": 60}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 56, 56, 60]}}
?
H	variables
Iregularization_losses
Jtrainable_variables
K	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "MaxPooling2D", "name": "max_pooling2d_34", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "max_pooling2d_34", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 4, "max_ndim": null, "min_ndim": null, "axes": {}}}}
?	

Lkernel
Mbias
N	variables
Oregularization_losses
Ptrainable_variables
Q	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Conv2D", "name": "conv2d_70", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "conv2d_70", "trainable": true, "dtype": "float32", "filters": 30, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 4, "axes": {"-1": 60}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 28, 28, 60]}}
?
R	variables
Sregularization_losses
Ttrainable_variables
U	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_121", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_121", "trainable": true, "dtype": "float32", "activation": "relu"}}
?	
Vaxis
	Wgamma
Xbeta
Ymoving_mean
Zmoving_variance
[	variables
\regularization_losses
]trainable_variables
^	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "BatchNormalization", "name": "batch_normalization_104", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "batch_normalization_104", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 4, "max_ndim": null, "min_ndim": null, "axes": {"3": 30}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 26, 26, 30]}}
?	

_kernel
`bias
a	variables
bregularization_losses
ctrainable_variables
d	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Conv2D", "name": "conv2d_71", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "conv2d_71", "trainable": true, "dtype": "float32", "filters": 30, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 4, "axes": {"-1": 30}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 26, 26, 30]}}
?
e	variables
fregularization_losses
gtrainable_variables
h	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_122", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_122", "trainable": true, "dtype": "float32", "activation": "relu"}}
?	
iaxis
	jgamma
kbeta
lmoving_mean
mmoving_variance
n	variables
oregularization_losses
ptrainable_variables
q	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "BatchNormalization", "name": "batch_normalization_105", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "batch_normalization_105", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 4, "max_ndim": null, "min_ndim": null, "axes": {"3": 30}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 24, 24, 30]}}
?
r	variables
sregularization_losses
ttrainable_variables
u	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "MaxPooling2D", "name": "max_pooling2d_35", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "max_pooling2d_35", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 4, "max_ndim": null, "min_ndim": null, "axes": {}}}}
?
v	variables
wregularization_losses
xtrainable_variables
y	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Dropout", "name": "dropout_51", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "dropout_51", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}
?
z	variables
{regularization_losses
|trainable_variables
}	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Flatten", "name": "flatten_34", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "flatten_34", "trainable": true, "dtype": "float32", "data_format": "channels_last"}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 1, "axes": {}}}}
?

~kernel
bias
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Dense", "name": "dense_51", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "dense_51", "trainable": true, "dtype": "float32", "units": 300, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 4320}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 4320]}}
?
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_123", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_123", "trainable": true, "dtype": "float32", "activation": "relu"}}
?	
	?axis

?gamma
	?beta
?moving_mean
?moving_variance
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "BatchNormalization", "name": "batch_normalization_106", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "batch_normalization_106", "trainable": true, "dtype": "float32", "axis": [1], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 2, "max_ndim": null, "min_ndim": null, "axes": {"1": 300}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 300]}}
?
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Dropout", "name": "dropout_52", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "dropout_52", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}
?
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Flatten", "name": "flatten_35", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "flatten_35", "trainable": true, "dtype": "float32", "data_format": "channels_last"}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 1, "axes": {}}}}
?
?kernel
	?bias
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Dense", "name": "dense_52", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "dense_52", "trainable": true, "dtype": "float32", "units": 300, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 300}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 300]}}
?
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_124", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_124", "trainable": true, "dtype": "float32", "activation": "relu"}}
?	
	?axis

?gamma
	?beta
?moving_mean
?moving_variance
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "BatchNormalization", "name": "batch_normalization_107", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "batch_normalization_107", "trainable": true, "dtype": "float32", "axis": [1], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 2, "max_ndim": null, "min_ndim": null, "axes": {"1": 300}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 300]}}
?
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Dropout", "name": "dropout_53", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "dropout_53", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}
?
?kernel
	?bias
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Dense", "name": "dense_53", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "dense_53", "trainable": true, "dtype": "float32", "units": 6, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 300}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 300]}}
?
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_125", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_125", "trainable": true, "dtype": "float32", "activation": "softmax"}}
?
	?iter
?beta_1
?beta_2

?decay
?learning_rate"m?#m?-m?.m?5m?6m?@m?Am?Lm?Mm?Wm?Xm?_m?`m?jm?km?~m?m?	?m?	?m?	?m?	?m?	?m?	?m?	?m?	?m?"v?#v?-v?.v?5v?6v?@v?Av?Lv?Mv?Wv?Xv?_v?`v?jv?kv?~v?v?	?v?	?v?	?v?	?v?	?v?	?v?	?v?	?v?"
	optimizer
?
"0
#1
-2
.3
/4
05
56
67
@8
A9
B10
C11
L12
M13
W14
X15
Y16
Z17
_18
`19
j20
k21
l22
m23
~24
25
?26
?27
?28
?29
?30
?31
?32
?33
?34
?35
?36
?37"
trackable_list_wrapper
 "
trackable_list_wrapper
?
"0
#1
-2
.3
54
65
@6
A7
L8
M9
W10
X11
_12
`13
j14
k15
~16
17
?18
?19
?20
?21
?22
?23
?24
?25"
trackable_list_wrapper
?
?metrics
	variables
?non_trainable_variables
regularization_losses
trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
?_default_save_signature
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
-
?serving_default"
signature_map
*:(<2conv2d_68/kernel
:<2conv2d_68/bias
.
"0
#1"
trackable_list_wrapper
 "
trackable_list_wrapper
.
"0
#1"
trackable_list_wrapper
?
?metrics
$	variables
?non_trainable_variables
%regularization_losses
&trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
?metrics
(	variables
?non_trainable_variables
)regularization_losses
*trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
+:)<2batch_normalization_102/gamma
*:(<2batch_normalization_102/beta
3:1< (2#batch_normalization_102/moving_mean
7:5< (2'batch_normalization_102/moving_variance
<
-0
.1
/2
03"
trackable_list_wrapper
 "
trackable_list_wrapper
.
-0
.1"
trackable_list_wrapper
?
?metrics
1	variables
?non_trainable_variables
2regularization_losses
3trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
*:(<<2conv2d_69/kernel
:<2conv2d_69/bias
.
50
61"
trackable_list_wrapper
 "
trackable_list_wrapper
.
50
61"
trackable_list_wrapper
?
?metrics
7	variables
?non_trainable_variables
8regularization_losses
9trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
?metrics
;	variables
?non_trainable_variables
<regularization_losses
=trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
+:)<2batch_normalization_103/gamma
*:(<2batch_normalization_103/beta
3:1< (2#batch_normalization_103/moving_mean
7:5< (2'batch_normalization_103/moving_variance
<
@0
A1
B2
C3"
trackable_list_wrapper
 "
trackable_list_wrapper
.
@0
A1"
trackable_list_wrapper
?
?metrics
D	variables
?non_trainable_variables
Eregularization_losses
Ftrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
?metrics
H	variables
?non_trainable_variables
Iregularization_losses
Jtrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
*:(<2conv2d_70/kernel
:2conv2d_70/bias
.
L0
M1"
trackable_list_wrapper
 "
trackable_list_wrapper
.
L0
M1"
trackable_list_wrapper
?
?metrics
N	variables
?non_trainable_variables
Oregularization_losses
Ptrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
?metrics
R	variables
?non_trainable_variables
Sregularization_losses
Ttrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
+:)2batch_normalization_104/gamma
*:(2batch_normalization_104/beta
3:1 (2#batch_normalization_104/moving_mean
7:5 (2'batch_normalization_104/moving_variance
<
W0
X1
Y2
Z3"
trackable_list_wrapper
 "
trackable_list_wrapper
.
W0
X1"
trackable_list_wrapper
?
?metrics
[	variables
?non_trainable_variables
\regularization_losses
]trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
*:(2conv2d_71/kernel
:2conv2d_71/bias
.
_0
`1"
trackable_list_wrapper
 "
trackable_list_wrapper
.
_0
`1"
trackable_list_wrapper
?
?metrics
a	variables
?non_trainable_variables
bregularization_losses
ctrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
?metrics
e	variables
?non_trainable_variables
fregularization_losses
gtrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
+:)2batch_normalization_105/gamma
*:(2batch_normalization_105/beta
3:1 (2#batch_normalization_105/moving_mean
7:5 (2'batch_normalization_105/moving_variance
<
j0
k1
l2
m3"
trackable_list_wrapper
 "
trackable_list_wrapper
.
j0
k1"
trackable_list_wrapper
?
?metrics
n	variables
?non_trainable_variables
oregularization_losses
ptrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
?metrics
r	variables
?non_trainable_variables
sregularization_losses
ttrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
?metrics
v	variables
?non_trainable_variables
wregularization_losses
xtrainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
?metrics
z	variables
?non_trainable_variables
{regularization_losses
|trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
#:!
?!?2dense_51/kernel
:?2dense_51/bias
.
~0
1"
trackable_list_wrapper
 "
trackable_list_wrapper
.
~0
1"
trackable_list_wrapper
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
,:*?2batch_normalization_106/gamma
+:)?2batch_normalization_106/beta
4:2? (2#batch_normalization_106/moving_mean
8:6? (2'batch_normalization_106/moving_variance
@
?0
?1
?2
?3"
trackable_list_wrapper
 "
trackable_list_wrapper
0
?0
?1"
trackable_list_wrapper
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
#:!
??2dense_52/kernel
:?2dense_52/bias
0
?0
?1"
trackable_list_wrapper
 "
trackable_list_wrapper
0
?0
?1"
trackable_list_wrapper
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
,:*?2batch_normalization_107/gamma
+:)?2batch_normalization_107/beta
4:2? (2#batch_normalization_107/moving_mean
8:6? (2'batch_normalization_107/moving_variance
@
?0
?1
?2
?3"
trackable_list_wrapper
 "
trackable_list_wrapper
0
?0
?1"
trackable_list_wrapper
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
": 	?2dense_53/kernel
:2dense_53/bias
0
?0
?1"
trackable_list_wrapper
 "
trackable_list_wrapper
0
?0
?1"
trackable_list_wrapper
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
?metrics
?	variables
?non_trainable_variables
?regularization_losses
?trainable_variables
?layers
 ?layer_regularization_losses
?layer_metrics
?__call__
+?&call_and_return_all_conditional_losses
'?"call_and_return_conditional_losses"
_generic_user_object
:	 (2	Adam/iter
: (2Adam/beta_1
: (2Adam/beta_2
: (2
Adam/decay
: (2Adam/learning_rate
0
?0
?1"
trackable_list_wrapper
z
/0
01
B2
C3
Y4
Z5
l6
m7
?8
?9
?10
?11"
trackable_list_wrapper
?
0
1
2
3
4
5
6
7
	8

9
10
11
12
13
14
15
16
17
18
19
20
21
22
23
24
25
26"
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
.
/0
01"
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
.
B0
C1"
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
.
Y0
Z1"
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
.
l0
m1"
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
0
?0
?1"
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
0
?0
?1"
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
?

?total

?count
?	variables
?	keras_api"?
_tf_keras_metricj{"class_name": "Mean", "name": "loss", "dtype": "float32", "config": {"name": "loss", "dtype": "float32"}}
?

?total

?count
?
_fn_kwargs
?	variables
?	keras_api"?
_tf_keras_metric?{"class_name": "MeanMetricWrapper", "name": "accuracy", "dtype": "float32", "config": {"name": "accuracy", "dtype": "float32", "fn": "categorical_accuracy"}}
:  (2total
:  (2count
0
?0
?1"
trackable_list_wrapper
.
?	variables"
_generic_user_object
:  (2total
:  (2count
 "
trackable_dict_wrapper
0
?0
?1"
trackable_list_wrapper
.
?	variables"
_generic_user_object
/:-<2Adam/conv2d_68/kernel/m
!:<2Adam/conv2d_68/bias/m
0:.<2$Adam/batch_normalization_102/gamma/m
/:-<2#Adam/batch_normalization_102/beta/m
/:-<<2Adam/conv2d_69/kernel/m
!:<2Adam/conv2d_69/bias/m
0:.<2$Adam/batch_normalization_103/gamma/m
/:-<2#Adam/batch_normalization_103/beta/m
/:-<2Adam/conv2d_70/kernel/m
!:2Adam/conv2d_70/bias/m
0:.2$Adam/batch_normalization_104/gamma/m
/:-2#Adam/batch_normalization_104/beta/m
/:-2Adam/conv2d_71/kernel/m
!:2Adam/conv2d_71/bias/m
0:.2$Adam/batch_normalization_105/gamma/m
/:-2#Adam/batch_normalization_105/beta/m
(:&
?!?2Adam/dense_51/kernel/m
!:?2Adam/dense_51/bias/m
1:/?2$Adam/batch_normalization_106/gamma/m
0:.?2#Adam/batch_normalization_106/beta/m
(:&
??2Adam/dense_52/kernel/m
!:?2Adam/dense_52/bias/m
1:/?2$Adam/batch_normalization_107/gamma/m
0:.?2#Adam/batch_normalization_107/beta/m
':%	?2Adam/dense_53/kernel/m
 :2Adam/dense_53/bias/m
/:-<2Adam/conv2d_68/kernel/v
!:<2Adam/conv2d_68/bias/v
0:.<2$Adam/batch_normalization_102/gamma/v
/:-<2#Adam/batch_normalization_102/beta/v
/:-<<2Adam/conv2d_69/kernel/v
!:<2Adam/conv2d_69/bias/v
0:.<2$Adam/batch_normalization_103/gamma/v
/:-<2#Adam/batch_normalization_103/beta/v
/:-<2Adam/conv2d_70/kernel/v
!:2Adam/conv2d_70/bias/v
0:.2$Adam/batch_normalization_104/gamma/v
/:-2#Adam/batch_normalization_104/beta/v
/:-2Adam/conv2d_71/kernel/v
!:2Adam/conv2d_71/bias/v
0:.2$Adam/batch_normalization_105/gamma/v
/:-2#Adam/batch_normalization_105/beta/v
(:&
?!?2Adam/dense_51/kernel/v
!:?2Adam/dense_51/bias/v
1:/?2$Adam/batch_normalization_106/gamma/v
0:.?2#Adam/batch_normalization_106/beta/v
(:&
??2Adam/dense_52/kernel/v
!:?2Adam/dense_52/bias/v
1:/?2$Adam/batch_normalization_107/gamma/v
0:.?2#Adam/batch_normalization_107/beta/v
':%	?2Adam/dense_53/kernel/v
 :2Adam/dense_53/bias/v
?2?
.__inference_sequential_17_layer_call_fn_584594
.__inference_sequential_17_layer_call_fn_585127
.__inference_sequential_17_layer_call_fn_584406
.__inference_sequential_17_layer_call_fn_585208?
???
FullArgSpec1
args)?&
jself
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults?
p 

 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
I__inference_sequential_17_layer_call_and_return_conditional_losses_584217
I__inference_sequential_17_layer_call_and_return_conditional_losses_584896
I__inference_sequential_17_layer_call_and_return_conditional_losses_584110
I__inference_sequential_17_layer_call_and_return_conditional_losses_585046?
???
FullArgSpec1
args)?&
jself
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults?
p 

 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
!__inference__wrapped_model_582630?
???
FullArgSpec
args? 
varargsjargs
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *6?3
1?.
conv2d_68_input?????????@@
?2?
*__inference_conv2d_68_layer_call_fn_585227?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
E__inference_conv2d_68_layer_call_and_return_conditional_losses_585218?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
/__inference_activation_119_layer_call_fn_585237?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
J__inference_activation_119_layer_call_and_return_conditional_losses_585232?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
8__inference_batch_normalization_102_layer_call_fn_585352
8__inference_batch_normalization_102_layer_call_fn_585365
8__inference_batch_normalization_102_layer_call_fn_585288
8__inference_batch_normalization_102_layer_call_fn_585301?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_585257
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_585275
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_585321
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_585339?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
*__inference_conv2d_69_layer_call_fn_585384?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
E__inference_conv2d_69_layer_call_and_return_conditional_losses_585375?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
/__inference_activation_120_layer_call_fn_585394?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
J__inference_activation_120_layer_call_and_return_conditional_losses_585389?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
8__inference_batch_normalization_103_layer_call_fn_585522
8__inference_batch_normalization_103_layer_call_fn_585509
8__inference_batch_normalization_103_layer_call_fn_585458
8__inference_batch_normalization_103_layer_call_fn_585445?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_585496
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_585478
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_585414
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_585432?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
1__inference_max_pooling2d_34_layer_call_fn_582850?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *@?=
;?84????????????????????????????????????
?2?
L__inference_max_pooling2d_34_layer_call_and_return_conditional_losses_582844?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *@?=
;?84????????????????????????????????????
?2?
*__inference_conv2d_70_layer_call_fn_585541?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
E__inference_conv2d_70_layer_call_and_return_conditional_losses_585532?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
/__inference_activation_121_layer_call_fn_585551?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
J__inference_activation_121_layer_call_and_return_conditional_losses_585546?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
8__inference_batch_normalization_104_layer_call_fn_585602
8__inference_batch_normalization_104_layer_call_fn_585615
8__inference_batch_normalization_104_layer_call_fn_585666
8__inference_batch_normalization_104_layer_call_fn_585679?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_585635
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_585589
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_585571
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_585653?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
*__inference_conv2d_71_layer_call_fn_585698?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
E__inference_conv2d_71_layer_call_and_return_conditional_losses_585689?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
/__inference_activation_122_layer_call_fn_585708?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
J__inference_activation_122_layer_call_and_return_conditional_losses_585703?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
8__inference_batch_normalization_105_layer_call_fn_585836
8__inference_batch_normalization_105_layer_call_fn_585772
8__inference_batch_normalization_105_layer_call_fn_585759
8__inference_batch_normalization_105_layer_call_fn_585823?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_585792
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_585810
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_585746
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_585728?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
1__inference_max_pooling2d_35_layer_call_fn_583070?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *@?=
;?84????????????????????????????????????
?2?
L__inference_max_pooling2d_35_layer_call_and_return_conditional_losses_583064?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *@?=
;?84????????????????????????????????????
?2?
+__inference_dropout_51_layer_call_fn_585858
+__inference_dropout_51_layer_call_fn_585863?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
F__inference_dropout_51_layer_call_and_return_conditional_losses_585853
F__inference_dropout_51_layer_call_and_return_conditional_losses_585848?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
+__inference_flatten_34_layer_call_fn_585874?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
F__inference_flatten_34_layer_call_and_return_conditional_losses_585869?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
)__inference_dense_51_layer_call_fn_585893?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
D__inference_dense_51_layer_call_and_return_conditional_losses_585884?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
/__inference_activation_123_layer_call_fn_585903?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
J__inference_activation_123_layer_call_and_return_conditional_losses_585898?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
8__inference_batch_normalization_106_layer_call_fn_585985
8__inference_batch_normalization_106_layer_call_fn_585972?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
S__inference_batch_normalization_106_layer_call_and_return_conditional_losses_585939
S__inference_batch_normalization_106_layer_call_and_return_conditional_losses_585959?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
+__inference_dropout_52_layer_call_fn_586007
+__inference_dropout_52_layer_call_fn_586012?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
F__inference_dropout_52_layer_call_and_return_conditional_losses_585997
F__inference_dropout_52_layer_call_and_return_conditional_losses_586002?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
+__inference_flatten_35_layer_call_fn_586023?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
F__inference_flatten_35_layer_call_and_return_conditional_losses_586018?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
)__inference_dense_52_layer_call_fn_586042?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
D__inference_dense_52_layer_call_and_return_conditional_losses_586033?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
/__inference_activation_124_layer_call_fn_586052?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
J__inference_activation_124_layer_call_and_return_conditional_losses_586047?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
8__inference_batch_normalization_107_layer_call_fn_586121
8__inference_batch_normalization_107_layer_call_fn_586134?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
S__inference_batch_normalization_107_layer_call_and_return_conditional_losses_586088
S__inference_batch_normalization_107_layer_call_and_return_conditional_losses_586108?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
+__inference_dropout_53_layer_call_fn_586161
+__inference_dropout_53_layer_call_fn_586156?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
F__inference_dropout_53_layer_call_and_return_conditional_losses_586151
F__inference_dropout_53_layer_call_and_return_conditional_losses_586146?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
)__inference_dense_53_layer_call_fn_586180?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
D__inference_dense_53_layer_call_and_return_conditional_losses_586171?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
/__inference_activation_125_layer_call_fn_586190?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
J__inference_activation_125_layer_call_and_return_conditional_losses_586185?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?B?
$__inference_signature_wrapper_584685conv2d_68_input"?
???
FullArgSpec
args? 
varargs
 
varkwjkwargs
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 ?
!__inference__wrapped_model_582630?2"#-./056@ABCLMWXYZ_`jklm~????????????@?=
6?3
1?.
conv2d_68_input?????????@@
? "??<
:
activation_125(?%
activation_125??????????
J__inference_activation_119_layer_call_and_return_conditional_losses_585232h7?4
-?*
(?%
inputs?????????<<<
? "-?*
#? 
0?????????<<<
? ?
/__inference_activation_119_layer_call_fn_585237[7?4
-?*
(?%
inputs?????????<<<
? " ??????????<<<?
J__inference_activation_120_layer_call_and_return_conditional_losses_585389h7?4
-?*
(?%
inputs?????????88<
? "-?*
#? 
0?????????88<
? ?
/__inference_activation_120_layer_call_fn_585394[7?4
-?*
(?%
inputs?????????88<
? " ??????????88<?
J__inference_activation_121_layer_call_and_return_conditional_losses_585546h7?4
-?*
(?%
inputs?????????
? "-?*
#? 
0?????????
? ?
/__inference_activation_121_layer_call_fn_585551[7?4
-?*
(?%
inputs?????????
? " ???????????
J__inference_activation_122_layer_call_and_return_conditional_losses_585703h7?4
-?*
(?%
inputs?????????
? "-?*
#? 
0?????????
? ?
/__inference_activation_122_layer_call_fn_585708[7?4
-?*
(?%
inputs?????????
? " ???????????
J__inference_activation_123_layer_call_and_return_conditional_losses_585898Z0?-
&?#
!?
inputs??????????
? "&?#
?
0??????????
? ?
/__inference_activation_123_layer_call_fn_585903M0?-
&?#
!?
inputs??????????
? "????????????
J__inference_activation_124_layer_call_and_return_conditional_losses_586047Z0?-
&?#
!?
inputs??????????
? "&?#
?
0??????????
? ?
/__inference_activation_124_layer_call_fn_586052M0?-
&?#
!?
inputs??????????
? "????????????
J__inference_activation_125_layer_call_and_return_conditional_losses_586185X/?,
%?"
 ?
inputs?????????
? "%?"
?
0?????????
? ~
/__inference_activation_125_layer_call_fn_586190K/?,
%?"
 ?
inputs?????????
? "???????????
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_585257?-./0M?J
C?@
:?7
inputs+???????????????????????????<
p
? "??<
5?2
0+???????????????????????????<
? ?
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_585275?-./0M?J
C?@
:?7
inputs+???????????????????????????<
p 
? "??<
5?2
0+???????????????????????????<
? ?
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_585321r-./0;?8
1?.
(?%
inputs?????????<<<
p
? "-?*
#? 
0?????????<<<
? ?
S__inference_batch_normalization_102_layer_call_and_return_conditional_losses_585339r-./0;?8
1?.
(?%
inputs?????????<<<
p 
? "-?*
#? 
0?????????<<<
? ?
8__inference_batch_normalization_102_layer_call_fn_585288?-./0M?J
C?@
:?7
inputs+???????????????????????????<
p
? "2?/+???????????????????????????<?
8__inference_batch_normalization_102_layer_call_fn_585301?-./0M?J
C?@
:?7
inputs+???????????????????????????<
p 
? "2?/+???????????????????????????<?
8__inference_batch_normalization_102_layer_call_fn_585352e-./0;?8
1?.
(?%
inputs?????????<<<
p
? " ??????????<<<?
8__inference_batch_normalization_102_layer_call_fn_585365e-./0;?8
1?.
(?%
inputs?????????<<<
p 
? " ??????????<<<?
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_585414r@ABC;?8
1?.
(?%
inputs?????????88<
p
? "-?*
#? 
0?????????88<
? ?
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_585432r@ABC;?8
1?.
(?%
inputs?????????88<
p 
? "-?*
#? 
0?????????88<
? ?
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_585478?@ABCM?J
C?@
:?7
inputs+???????????????????????????<
p
? "??<
5?2
0+???????????????????????????<
? ?
S__inference_batch_normalization_103_layer_call_and_return_conditional_losses_585496?@ABCM?J
C?@
:?7
inputs+???????????????????????????<
p 
? "??<
5?2
0+???????????????????????????<
? ?
8__inference_batch_normalization_103_layer_call_fn_585445e@ABC;?8
1?.
(?%
inputs?????????88<
p
? " ??????????88<?
8__inference_batch_normalization_103_layer_call_fn_585458e@ABC;?8
1?.
(?%
inputs?????????88<
p 
? " ??????????88<?
8__inference_batch_normalization_103_layer_call_fn_585509?@ABCM?J
C?@
:?7
inputs+???????????????????????????<
p
? "2?/+???????????????????????????<?
8__inference_batch_normalization_103_layer_call_fn_585522?@ABCM?J
C?@
:?7
inputs+???????????????????????????<
p 
? "2?/+???????????????????????????<?
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_585571?WXYZM?J
C?@
:?7
inputs+???????????????????????????
p
? "??<
5?2
0+???????????????????????????
? ?
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_585589?WXYZM?J
C?@
:?7
inputs+???????????????????????????
p 
? "??<
5?2
0+???????????????????????????
? ?
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_585635rWXYZ;?8
1?.
(?%
inputs?????????
p
? "-?*
#? 
0?????????
? ?
S__inference_batch_normalization_104_layer_call_and_return_conditional_losses_585653rWXYZ;?8
1?.
(?%
inputs?????????
p 
? "-?*
#? 
0?????????
? ?
8__inference_batch_normalization_104_layer_call_fn_585602?WXYZM?J
C?@
:?7
inputs+???????????????????????????
p
? "2?/+????????????????????????????
8__inference_batch_normalization_104_layer_call_fn_585615?WXYZM?J
C?@
:?7
inputs+???????????????????????????
p 
? "2?/+????????????????????????????
8__inference_batch_normalization_104_layer_call_fn_585666eWXYZ;?8
1?.
(?%
inputs?????????
p
? " ???????????
8__inference_batch_normalization_104_layer_call_fn_585679eWXYZ;?8
1?.
(?%
inputs?????????
p 
? " ???????????
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_585728rjklm;?8
1?.
(?%
inputs?????????
p
? "-?*
#? 
0?????????
? ?
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_585746rjklm;?8
1?.
(?%
inputs?????????
p 
? "-?*
#? 
0?????????
? ?
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_585792?jklmM?J
C?@
:?7
inputs+???????????????????????????
p
? "??<
5?2
0+???????????????????????????
? ?
S__inference_batch_normalization_105_layer_call_and_return_conditional_losses_585810?jklmM?J
C?@
:?7
inputs+???????????????????????????
p 
? "??<
5?2
0+???????????????????????????
? ?
8__inference_batch_normalization_105_layer_call_fn_585759ejklm;?8
1?.
(?%
inputs?????????
p
? " ???????????
8__inference_batch_normalization_105_layer_call_fn_585772ejklm;?8
1?.
(?%
inputs?????????
p 
? " ???????????
8__inference_batch_normalization_105_layer_call_fn_585823?jklmM?J
C?@
:?7
inputs+???????????????????????????
p
? "2?/+????????????????????????????
8__inference_batch_normalization_105_layer_call_fn_585836?jklmM?J
C?@
:?7
inputs+???????????????????????????
p 
? "2?/+????????????????????????????
S__inference_batch_normalization_106_layer_call_and_return_conditional_losses_585939h????4?1
*?'
!?
inputs??????????
p
? "&?#
?
0??????????
? ?
S__inference_batch_normalization_106_layer_call_and_return_conditional_losses_585959h????4?1
*?'
!?
inputs??????????
p 
? "&?#
?
0??????????
? ?
8__inference_batch_normalization_106_layer_call_fn_585972[????4?1
*?'
!?
inputs??????????
p
? "????????????
8__inference_batch_normalization_106_layer_call_fn_585985[????4?1
*?'
!?
inputs??????????
p 
? "????????????
S__inference_batch_normalization_107_layer_call_and_return_conditional_losses_586088h????4?1
*?'
!?
inputs??????????
p
? "&?#
?
0??????????
? ?
S__inference_batch_normalization_107_layer_call_and_return_conditional_losses_586108h????4?1
*?'
!?
inputs??????????
p 
? "&?#
?
0??????????
? ?
8__inference_batch_normalization_107_layer_call_fn_586121[????4?1
*?'
!?
inputs??????????
p
? "????????????
8__inference_batch_normalization_107_layer_call_fn_586134[????4?1
*?'
!?
inputs??????????
p 
? "????????????
E__inference_conv2d_68_layer_call_and_return_conditional_losses_585218l"#7?4
-?*
(?%
inputs?????????@@
? "-?*
#? 
0?????????<<<
? ?
*__inference_conv2d_68_layer_call_fn_585227_"#7?4
-?*
(?%
inputs?????????@@
? " ??????????<<<?
E__inference_conv2d_69_layer_call_and_return_conditional_losses_585375l567?4
-?*
(?%
inputs?????????<<<
? "-?*
#? 
0?????????88<
? ?
*__inference_conv2d_69_layer_call_fn_585384_567?4
-?*
(?%
inputs?????????<<<
? " ??????????88<?
E__inference_conv2d_70_layer_call_and_return_conditional_losses_585532lLM7?4
-?*
(?%
inputs?????????<
? "-?*
#? 
0?????????
? ?
*__inference_conv2d_70_layer_call_fn_585541_LM7?4
-?*
(?%
inputs?????????<
? " ???????????
E__inference_conv2d_71_layer_call_and_return_conditional_losses_585689l_`7?4
-?*
(?%
inputs?????????
? "-?*
#? 
0?????????
? ?
*__inference_conv2d_71_layer_call_fn_585698__`7?4
-?*
(?%
inputs?????????
? " ???????????
D__inference_dense_51_layer_call_and_return_conditional_losses_585884^~0?-
&?#
!?
inputs??????????!
? "&?#
?
0??????????
? ~
)__inference_dense_51_layer_call_fn_585893Q~0?-
&?#
!?
inputs??????????!
? "????????????
D__inference_dense_52_layer_call_and_return_conditional_losses_586033`??0?-
&?#
!?
inputs??????????
? "&?#
?
0??????????
? ?
)__inference_dense_52_layer_call_fn_586042S??0?-
&?#
!?
inputs??????????
? "????????????
D__inference_dense_53_layer_call_and_return_conditional_losses_586171_??0?-
&?#
!?
inputs??????????
? "%?"
?
0?????????
? 
)__inference_dense_53_layer_call_fn_586180R??0?-
&?#
!?
inputs??????????
? "???????????
F__inference_dropout_51_layer_call_and_return_conditional_losses_585848l;?8
1?.
(?%
inputs?????????
p
? "-?*
#? 
0?????????
? ?
F__inference_dropout_51_layer_call_and_return_conditional_losses_585853l;?8
1?.
(?%
inputs?????????
p 
? "-?*
#? 
0?????????
? ?
+__inference_dropout_51_layer_call_fn_585858_;?8
1?.
(?%
inputs?????????
p
? " ???????????
+__inference_dropout_51_layer_call_fn_585863_;?8
1?.
(?%
inputs?????????
p 
? " ???????????
F__inference_dropout_52_layer_call_and_return_conditional_losses_585997^4?1
*?'
!?
inputs??????????
p
? "&?#
?
0??????????
? ?
F__inference_dropout_52_layer_call_and_return_conditional_losses_586002^4?1
*?'
!?
inputs??????????
p 
? "&?#
?
0??????????
? ?
+__inference_dropout_52_layer_call_fn_586007Q4?1
*?'
!?
inputs??????????
p
? "????????????
+__inference_dropout_52_layer_call_fn_586012Q4?1
*?'
!?
inputs??????????
p 
? "????????????
F__inference_dropout_53_layer_call_and_return_conditional_losses_586146^4?1
*?'
!?
inputs??????????
p
? "&?#
?
0??????????
? ?
F__inference_dropout_53_layer_call_and_return_conditional_losses_586151^4?1
*?'
!?
inputs??????????
p 
? "&?#
?
0??????????
? ?
+__inference_dropout_53_layer_call_fn_586156Q4?1
*?'
!?
inputs??????????
p
? "????????????
+__inference_dropout_53_layer_call_fn_586161Q4?1
*?'
!?
inputs??????????
p 
? "????????????
F__inference_flatten_34_layer_call_and_return_conditional_losses_585869a7?4
-?*
(?%
inputs?????????
? "&?#
?
0??????????!
? ?
+__inference_flatten_34_layer_call_fn_585874T7?4
-?*
(?%
inputs?????????
? "???????????!?
F__inference_flatten_35_layer_call_and_return_conditional_losses_586018Z0?-
&?#
!?
inputs??????????
? "&?#
?
0??????????
? |
+__inference_flatten_35_layer_call_fn_586023M0?-
&?#
!?
inputs??????????
? "????????????
L__inference_max_pooling2d_34_layer_call_and_return_conditional_losses_582844?R?O
H?E
C?@
inputs4????????????????????????????????????
? "H?E
>?;
04????????????????????????????????????
? ?
1__inference_max_pooling2d_34_layer_call_fn_582850?R?O
H?E
C?@
inputs4????????????????????????????????????
? ";?84?????????????????????????????????????
L__inference_max_pooling2d_35_layer_call_and_return_conditional_losses_583064?R?O
H?E
C?@
inputs4????????????????????????????????????
? "H?E
>?;
04????????????????????????????????????
? ?
1__inference_max_pooling2d_35_layer_call_fn_583070?R?O
H?E
C?@
inputs4????????????????????????????????????
? ";?84?????????????????????????????????????
I__inference_sequential_17_layer_call_and_return_conditional_losses_584110?2"#-./056@ABCLMWXYZ_`jklm~????????????H?E
>?;
1?.
conv2d_68_input?????????@@
p

 
? "%?"
?
0?????????
? ?
I__inference_sequential_17_layer_call_and_return_conditional_losses_584217?2"#-./056@ABCLMWXYZ_`jklm~????????????H?E
>?;
1?.
conv2d_68_input?????????@@
p 

 
? "%?"
?
0?????????
? ?
I__inference_sequential_17_layer_call_and_return_conditional_losses_584896?2"#-./056@ABCLMWXYZ_`jklm~??????????????<
5?2
(?%
inputs?????????@@
p

 
? "%?"
?
0?????????
? ?
I__inference_sequential_17_layer_call_and_return_conditional_losses_585046?2"#-./056@ABCLMWXYZ_`jklm~??????????????<
5?2
(?%
inputs?????????@@
p 

 
? "%?"
?
0?????????
? ?
.__inference_sequential_17_layer_call_fn_584406?2"#-./056@ABCLMWXYZ_`jklm~????????????H?E
>?;
1?.
conv2d_68_input?????????@@
p

 
? "???????????
.__inference_sequential_17_layer_call_fn_584594?2"#-./056@ABCLMWXYZ_`jklm~????????????H?E
>?;
1?.
conv2d_68_input?????????@@
p 

 
? "???????????
.__inference_sequential_17_layer_call_fn_585127?2"#-./056@ABCLMWXYZ_`jklm~??????????????<
5?2
(?%
inputs?????????@@
p

 
? "???????????
.__inference_sequential_17_layer_call_fn_585208?2"#-./056@ABCLMWXYZ_`jklm~??????????????<
5?2
(?%
inputs?????????@@
p 

 
? "???????????
$__inference_signature_wrapper_584685?2"#-./056@ABCLMWXYZ_`jklm~????????????S?P
? 
I?F
D
conv2d_68_input1?.
conv2d_68_input?????????@@"??<
:
activation_125(?%
activation_125?????????