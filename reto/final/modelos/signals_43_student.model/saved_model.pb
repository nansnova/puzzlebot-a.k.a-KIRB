??&
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
conv2d_112/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*"
shared_nameconv2d_112/kernel

%conv2d_112/kernel/Read/ReadVariableOpReadVariableOpconv2d_112/kernel*&
_output_shapes
:<*
dtype0
v
conv2d_112/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:<* 
shared_nameconv2d_112/bias
o
#conv2d_112/bias/Read/ReadVariableOpReadVariableOpconv2d_112/bias*
_output_shapes
:<*
dtype0
?
batch_normalization_168/gammaVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*.
shared_namebatch_normalization_168/gamma
?
1batch_normalization_168/gamma/Read/ReadVariableOpReadVariableOpbatch_normalization_168/gamma*
_output_shapes
:<*
dtype0
?
batch_normalization_168/betaVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*-
shared_namebatch_normalization_168/beta
?
0batch_normalization_168/beta/Read/ReadVariableOpReadVariableOpbatch_normalization_168/beta*
_output_shapes
:<*
dtype0
?
#batch_normalization_168/moving_meanVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*4
shared_name%#batch_normalization_168/moving_mean
?
7batch_normalization_168/moving_mean/Read/ReadVariableOpReadVariableOp#batch_normalization_168/moving_mean*
_output_shapes
:<*
dtype0
?
'batch_normalization_168/moving_varianceVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*8
shared_name)'batch_normalization_168/moving_variance
?
;batch_normalization_168/moving_variance/Read/ReadVariableOpReadVariableOp'batch_normalization_168/moving_variance*
_output_shapes
:<*
dtype0
?
conv2d_113/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:<<*"
shared_nameconv2d_113/kernel

%conv2d_113/kernel/Read/ReadVariableOpReadVariableOpconv2d_113/kernel*&
_output_shapes
:<<*
dtype0
v
conv2d_113/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:<* 
shared_nameconv2d_113/bias
o
#conv2d_113/bias/Read/ReadVariableOpReadVariableOpconv2d_113/bias*
_output_shapes
:<*
dtype0
?
batch_normalization_169/gammaVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*.
shared_namebatch_normalization_169/gamma
?
1batch_normalization_169/gamma/Read/ReadVariableOpReadVariableOpbatch_normalization_169/gamma*
_output_shapes
:<*
dtype0
?
batch_normalization_169/betaVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*-
shared_namebatch_normalization_169/beta
?
0batch_normalization_169/beta/Read/ReadVariableOpReadVariableOpbatch_normalization_169/beta*
_output_shapes
:<*
dtype0
?
#batch_normalization_169/moving_meanVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*4
shared_name%#batch_normalization_169/moving_mean
?
7batch_normalization_169/moving_mean/Read/ReadVariableOpReadVariableOp#batch_normalization_169/moving_mean*
_output_shapes
:<*
dtype0
?
'batch_normalization_169/moving_varianceVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*8
shared_name)'batch_normalization_169/moving_variance
?
;batch_normalization_169/moving_variance/Read/ReadVariableOpReadVariableOp'batch_normalization_169/moving_variance*
_output_shapes
:<*
dtype0
?
conv2d_114/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*"
shared_nameconv2d_114/kernel

%conv2d_114/kernel/Read/ReadVariableOpReadVariableOpconv2d_114/kernel*&
_output_shapes
:<*
dtype0
v
conv2d_114/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:* 
shared_nameconv2d_114/bias
o
#conv2d_114/bias/Read/ReadVariableOpReadVariableOpconv2d_114/bias*
_output_shapes
:*
dtype0
?
batch_normalization_170/gammaVarHandleOp*
_output_shapes
: *
dtype0*
shape:*.
shared_namebatch_normalization_170/gamma
?
1batch_normalization_170/gamma/Read/ReadVariableOpReadVariableOpbatch_normalization_170/gamma*
_output_shapes
:*
dtype0
?
batch_normalization_170/betaVarHandleOp*
_output_shapes
: *
dtype0*
shape:*-
shared_namebatch_normalization_170/beta
?
0batch_normalization_170/beta/Read/ReadVariableOpReadVariableOpbatch_normalization_170/beta*
_output_shapes
:*
dtype0
?
#batch_normalization_170/moving_meanVarHandleOp*
_output_shapes
: *
dtype0*
shape:*4
shared_name%#batch_normalization_170/moving_mean
?
7batch_normalization_170/moving_mean/Read/ReadVariableOpReadVariableOp#batch_normalization_170/moving_mean*
_output_shapes
:*
dtype0
?
'batch_normalization_170/moving_varianceVarHandleOp*
_output_shapes
: *
dtype0*
shape:*8
shared_name)'batch_normalization_170/moving_variance
?
;batch_normalization_170/moving_variance/Read/ReadVariableOpReadVariableOp'batch_normalization_170/moving_variance*
_output_shapes
:*
dtype0
?
conv2d_115/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:*"
shared_nameconv2d_115/kernel

%conv2d_115/kernel/Read/ReadVariableOpReadVariableOpconv2d_115/kernel*&
_output_shapes
:*
dtype0
v
conv2d_115/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:* 
shared_nameconv2d_115/bias
o
#conv2d_115/bias/Read/ReadVariableOpReadVariableOpconv2d_115/bias*
_output_shapes
:*
dtype0
?
batch_normalization_171/gammaVarHandleOp*
_output_shapes
: *
dtype0*
shape:*.
shared_namebatch_normalization_171/gamma
?
1batch_normalization_171/gamma/Read/ReadVariableOpReadVariableOpbatch_normalization_171/gamma*
_output_shapes
:*
dtype0
?
batch_normalization_171/betaVarHandleOp*
_output_shapes
: *
dtype0*
shape:*-
shared_namebatch_normalization_171/beta
?
0batch_normalization_171/beta/Read/ReadVariableOpReadVariableOpbatch_normalization_171/beta*
_output_shapes
:*
dtype0
?
#batch_normalization_171/moving_meanVarHandleOp*
_output_shapes
: *
dtype0*
shape:*4
shared_name%#batch_normalization_171/moving_mean
?
7batch_normalization_171/moving_mean/Read/ReadVariableOpReadVariableOp#batch_normalization_171/moving_mean*
_output_shapes
:*
dtype0
?
'batch_normalization_171/moving_varianceVarHandleOp*
_output_shapes
: *
dtype0*
shape:*8
shared_name)'batch_normalization_171/moving_variance
?
;batch_normalization_171/moving_variance/Read/ReadVariableOpReadVariableOp'batch_normalization_171/moving_variance*
_output_shapes
:*
dtype0
|
dense_84/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:
?!?* 
shared_namedense_84/kernel
u
#dense_84/kernel/Read/ReadVariableOpReadVariableOpdense_84/kernel* 
_output_shapes
:
?!?*
dtype0
s
dense_84/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*
shared_namedense_84/bias
l
!dense_84/bias/Read/ReadVariableOpReadVariableOpdense_84/bias*
_output_shapes	
:?*
dtype0
?
batch_normalization_172/gammaVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*.
shared_namebatch_normalization_172/gamma
?
1batch_normalization_172/gamma/Read/ReadVariableOpReadVariableOpbatch_normalization_172/gamma*
_output_shapes	
:?*
dtype0
?
batch_normalization_172/betaVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*-
shared_namebatch_normalization_172/beta
?
0batch_normalization_172/beta/Read/ReadVariableOpReadVariableOpbatch_normalization_172/beta*
_output_shapes	
:?*
dtype0
?
#batch_normalization_172/moving_meanVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*4
shared_name%#batch_normalization_172/moving_mean
?
7batch_normalization_172/moving_mean/Read/ReadVariableOpReadVariableOp#batch_normalization_172/moving_mean*
_output_shapes	
:?*
dtype0
?
'batch_normalization_172/moving_varianceVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*8
shared_name)'batch_normalization_172/moving_variance
?
;batch_normalization_172/moving_variance/Read/ReadVariableOpReadVariableOp'batch_normalization_172/moving_variance*
_output_shapes	
:?*
dtype0
|
dense_85/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:
??* 
shared_namedense_85/kernel
u
#dense_85/kernel/Read/ReadVariableOpReadVariableOpdense_85/kernel* 
_output_shapes
:
??*
dtype0
s
dense_85/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*
shared_namedense_85/bias
l
!dense_85/bias/Read/ReadVariableOpReadVariableOpdense_85/bias*
_output_shapes	
:?*
dtype0
?
batch_normalization_173/gammaVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*.
shared_namebatch_normalization_173/gamma
?
1batch_normalization_173/gamma/Read/ReadVariableOpReadVariableOpbatch_normalization_173/gamma*
_output_shapes	
:?*
dtype0
?
batch_normalization_173/betaVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*-
shared_namebatch_normalization_173/beta
?
0batch_normalization_173/beta/Read/ReadVariableOpReadVariableOpbatch_normalization_173/beta*
_output_shapes	
:?*
dtype0
?
#batch_normalization_173/moving_meanVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*4
shared_name%#batch_normalization_173/moving_mean
?
7batch_normalization_173/moving_mean/Read/ReadVariableOpReadVariableOp#batch_normalization_173/moving_mean*
_output_shapes	
:?*
dtype0
?
'batch_normalization_173/moving_varianceVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*8
shared_name)'batch_normalization_173/moving_variance
?
;batch_normalization_173/moving_variance/Read/ReadVariableOpReadVariableOp'batch_normalization_173/moving_variance*
_output_shapes	
:?*
dtype0
{
dense_86/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:	?* 
shared_namedense_86/kernel
t
#dense_86/kernel/Read/ReadVariableOpReadVariableOpdense_86/kernel*
_output_shapes
:	?*
dtype0
r
dense_86/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:*
shared_namedense_86/bias
k
!dense_86/bias/Read/ReadVariableOpReadVariableOpdense_86/bias*
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
Adam/conv2d_112/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*)
shared_nameAdam/conv2d_112/kernel/m
?
,Adam/conv2d_112/kernel/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_112/kernel/m*&
_output_shapes
:<*
dtype0
?
Adam/conv2d_112/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*'
shared_nameAdam/conv2d_112/bias/m
}
*Adam/conv2d_112/bias/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_112/bias/m*
_output_shapes
:<*
dtype0
?
$Adam/batch_normalization_168/gamma/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*5
shared_name&$Adam/batch_normalization_168/gamma/m
?
8Adam/batch_normalization_168/gamma/m/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_168/gamma/m*
_output_shapes
:<*
dtype0
?
#Adam/batch_normalization_168/beta/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*4
shared_name%#Adam/batch_normalization_168/beta/m
?
7Adam/batch_normalization_168/beta/m/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_168/beta/m*
_output_shapes
:<*
dtype0
?
Adam/conv2d_113/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<<*)
shared_nameAdam/conv2d_113/kernel/m
?
,Adam/conv2d_113/kernel/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_113/kernel/m*&
_output_shapes
:<<*
dtype0
?
Adam/conv2d_113/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*'
shared_nameAdam/conv2d_113/bias/m
}
*Adam/conv2d_113/bias/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_113/bias/m*
_output_shapes
:<*
dtype0
?
$Adam/batch_normalization_169/gamma/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*5
shared_name&$Adam/batch_normalization_169/gamma/m
?
8Adam/batch_normalization_169/gamma/m/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_169/gamma/m*
_output_shapes
:<*
dtype0
?
#Adam/batch_normalization_169/beta/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*4
shared_name%#Adam/batch_normalization_169/beta/m
?
7Adam/batch_normalization_169/beta/m/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_169/beta/m*
_output_shapes
:<*
dtype0
?
Adam/conv2d_114/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*)
shared_nameAdam/conv2d_114/kernel/m
?
,Adam/conv2d_114/kernel/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_114/kernel/m*&
_output_shapes
:<*
dtype0
?
Adam/conv2d_114/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*'
shared_nameAdam/conv2d_114/bias/m
}
*Adam/conv2d_114/bias/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_114/bias/m*
_output_shapes
:*
dtype0
?
$Adam/batch_normalization_170/gamma/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*5
shared_name&$Adam/batch_normalization_170/gamma/m
?
8Adam/batch_normalization_170/gamma/m/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_170/gamma/m*
_output_shapes
:*
dtype0
?
#Adam/batch_normalization_170/beta/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*4
shared_name%#Adam/batch_normalization_170/beta/m
?
7Adam/batch_normalization_170/beta/m/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_170/beta/m*
_output_shapes
:*
dtype0
?
Adam/conv2d_115/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*)
shared_nameAdam/conv2d_115/kernel/m
?
,Adam/conv2d_115/kernel/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_115/kernel/m*&
_output_shapes
:*
dtype0
?
Adam/conv2d_115/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*'
shared_nameAdam/conv2d_115/bias/m
}
*Adam/conv2d_115/bias/m/Read/ReadVariableOpReadVariableOpAdam/conv2d_115/bias/m*
_output_shapes
:*
dtype0
?
$Adam/batch_normalization_171/gamma/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*5
shared_name&$Adam/batch_normalization_171/gamma/m
?
8Adam/batch_normalization_171/gamma/m/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_171/gamma/m*
_output_shapes
:*
dtype0
?
#Adam/batch_normalization_171/beta/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*4
shared_name%#Adam/batch_normalization_171/beta/m
?
7Adam/batch_normalization_171/beta/m/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_171/beta/m*
_output_shapes
:*
dtype0
?
Adam/dense_84/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:
?!?*'
shared_nameAdam/dense_84/kernel/m
?
*Adam/dense_84/kernel/m/Read/ReadVariableOpReadVariableOpAdam/dense_84/kernel/m* 
_output_shapes
:
?!?*
dtype0
?
Adam/dense_84/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*%
shared_nameAdam/dense_84/bias/m
z
(Adam/dense_84/bias/m/Read/ReadVariableOpReadVariableOpAdam/dense_84/bias/m*
_output_shapes	
:?*
dtype0
?
$Adam/batch_normalization_172/gamma/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*5
shared_name&$Adam/batch_normalization_172/gamma/m
?
8Adam/batch_normalization_172/gamma/m/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_172/gamma/m*
_output_shapes	
:?*
dtype0
?
#Adam/batch_normalization_172/beta/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*4
shared_name%#Adam/batch_normalization_172/beta/m
?
7Adam/batch_normalization_172/beta/m/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_172/beta/m*
_output_shapes	
:?*
dtype0
?
Adam/dense_85/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:
??*'
shared_nameAdam/dense_85/kernel/m
?
*Adam/dense_85/kernel/m/Read/ReadVariableOpReadVariableOpAdam/dense_85/kernel/m* 
_output_shapes
:
??*
dtype0
?
Adam/dense_85/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*%
shared_nameAdam/dense_85/bias/m
z
(Adam/dense_85/bias/m/Read/ReadVariableOpReadVariableOpAdam/dense_85/bias/m*
_output_shapes	
:?*
dtype0
?
$Adam/batch_normalization_173/gamma/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*5
shared_name&$Adam/batch_normalization_173/gamma/m
?
8Adam/batch_normalization_173/gamma/m/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_173/gamma/m*
_output_shapes	
:?*
dtype0
?
#Adam/batch_normalization_173/beta/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*4
shared_name%#Adam/batch_normalization_173/beta/m
?
7Adam/batch_normalization_173/beta/m/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_173/beta/m*
_output_shapes	
:?*
dtype0
?
Adam/dense_86/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:	?*'
shared_nameAdam/dense_86/kernel/m
?
*Adam/dense_86/kernel/m/Read/ReadVariableOpReadVariableOpAdam/dense_86/kernel/m*
_output_shapes
:	?*
dtype0
?
Adam/dense_86/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*%
shared_nameAdam/dense_86/bias/m
y
(Adam/dense_86/bias/m/Read/ReadVariableOpReadVariableOpAdam/dense_86/bias/m*
_output_shapes
:*
dtype0
?
Adam/conv2d_112/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*)
shared_nameAdam/conv2d_112/kernel/v
?
,Adam/conv2d_112/kernel/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_112/kernel/v*&
_output_shapes
:<*
dtype0
?
Adam/conv2d_112/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*'
shared_nameAdam/conv2d_112/bias/v
}
*Adam/conv2d_112/bias/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_112/bias/v*
_output_shapes
:<*
dtype0
?
$Adam/batch_normalization_168/gamma/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*5
shared_name&$Adam/batch_normalization_168/gamma/v
?
8Adam/batch_normalization_168/gamma/v/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_168/gamma/v*
_output_shapes
:<*
dtype0
?
#Adam/batch_normalization_168/beta/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*4
shared_name%#Adam/batch_normalization_168/beta/v
?
7Adam/batch_normalization_168/beta/v/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_168/beta/v*
_output_shapes
:<*
dtype0
?
Adam/conv2d_113/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<<*)
shared_nameAdam/conv2d_113/kernel/v
?
,Adam/conv2d_113/kernel/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_113/kernel/v*&
_output_shapes
:<<*
dtype0
?
Adam/conv2d_113/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*'
shared_nameAdam/conv2d_113/bias/v
}
*Adam/conv2d_113/bias/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_113/bias/v*
_output_shapes
:<*
dtype0
?
$Adam/batch_normalization_169/gamma/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*5
shared_name&$Adam/batch_normalization_169/gamma/v
?
8Adam/batch_normalization_169/gamma/v/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_169/gamma/v*
_output_shapes
:<*
dtype0
?
#Adam/batch_normalization_169/beta/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*4
shared_name%#Adam/batch_normalization_169/beta/v
?
7Adam/batch_normalization_169/beta/v/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_169/beta/v*
_output_shapes
:<*
dtype0
?
Adam/conv2d_114/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:<*)
shared_nameAdam/conv2d_114/kernel/v
?
,Adam/conv2d_114/kernel/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_114/kernel/v*&
_output_shapes
:<*
dtype0
?
Adam/conv2d_114/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*'
shared_nameAdam/conv2d_114/bias/v
}
*Adam/conv2d_114/bias/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_114/bias/v*
_output_shapes
:*
dtype0
?
$Adam/batch_normalization_170/gamma/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*5
shared_name&$Adam/batch_normalization_170/gamma/v
?
8Adam/batch_normalization_170/gamma/v/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_170/gamma/v*
_output_shapes
:*
dtype0
?
#Adam/batch_normalization_170/beta/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*4
shared_name%#Adam/batch_normalization_170/beta/v
?
7Adam/batch_normalization_170/beta/v/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_170/beta/v*
_output_shapes
:*
dtype0
?
Adam/conv2d_115/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*)
shared_nameAdam/conv2d_115/kernel/v
?
,Adam/conv2d_115/kernel/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_115/kernel/v*&
_output_shapes
:*
dtype0
?
Adam/conv2d_115/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*'
shared_nameAdam/conv2d_115/bias/v
}
*Adam/conv2d_115/bias/v/Read/ReadVariableOpReadVariableOpAdam/conv2d_115/bias/v*
_output_shapes
:*
dtype0
?
$Adam/batch_normalization_171/gamma/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*5
shared_name&$Adam/batch_normalization_171/gamma/v
?
8Adam/batch_normalization_171/gamma/v/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_171/gamma/v*
_output_shapes
:*
dtype0
?
#Adam/batch_normalization_171/beta/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*4
shared_name%#Adam/batch_normalization_171/beta/v
?
7Adam/batch_normalization_171/beta/v/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_171/beta/v*
_output_shapes
:*
dtype0
?
Adam/dense_84/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:
?!?*'
shared_nameAdam/dense_84/kernel/v
?
*Adam/dense_84/kernel/v/Read/ReadVariableOpReadVariableOpAdam/dense_84/kernel/v* 
_output_shapes
:
?!?*
dtype0
?
Adam/dense_84/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*%
shared_nameAdam/dense_84/bias/v
z
(Adam/dense_84/bias/v/Read/ReadVariableOpReadVariableOpAdam/dense_84/bias/v*
_output_shapes	
:?*
dtype0
?
$Adam/batch_normalization_172/gamma/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*5
shared_name&$Adam/batch_normalization_172/gamma/v
?
8Adam/batch_normalization_172/gamma/v/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_172/gamma/v*
_output_shapes	
:?*
dtype0
?
#Adam/batch_normalization_172/beta/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*4
shared_name%#Adam/batch_normalization_172/beta/v
?
7Adam/batch_normalization_172/beta/v/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_172/beta/v*
_output_shapes	
:?*
dtype0
?
Adam/dense_85/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:
??*'
shared_nameAdam/dense_85/kernel/v
?
*Adam/dense_85/kernel/v/Read/ReadVariableOpReadVariableOpAdam/dense_85/kernel/v* 
_output_shapes
:
??*
dtype0
?
Adam/dense_85/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*%
shared_nameAdam/dense_85/bias/v
z
(Adam/dense_85/bias/v/Read/ReadVariableOpReadVariableOpAdam/dense_85/bias/v*
_output_shapes	
:?*
dtype0
?
$Adam/batch_normalization_173/gamma/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*5
shared_name&$Adam/batch_normalization_173/gamma/v
?
8Adam/batch_normalization_173/gamma/v/Read/ReadVariableOpReadVariableOp$Adam/batch_normalization_173/gamma/v*
_output_shapes	
:?*
dtype0
?
#Adam/batch_normalization_173/beta/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:?*4
shared_name%#Adam/batch_normalization_173/beta/v
?
7Adam/batch_normalization_173/beta/v/Read/ReadVariableOpReadVariableOp#Adam/batch_normalization_173/beta/v*
_output_shapes	
:?*
dtype0
?
Adam/dense_86/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:	?*'
shared_nameAdam/dense_86/kernel/v
?
*Adam/dense_86/kernel/v/Read/ReadVariableOpReadVariableOpAdam/dense_86/kernel/v*
_output_shapes
:	?*
dtype0
?
Adam/dense_86/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*%
shared_nameAdam/dense_86/bias/v
y
(Adam/dense_86/bias/v/Read/ReadVariableOpReadVariableOpAdam/dense_86/bias/v*
_output_shapes
:*
dtype0

NoOpNoOp
??
ConstConst"/device:CPU:0*
_output_shapes
: *
dtype0*͵
valueµB?? B??
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
][
VARIABLE_VALUEconv2d_112/kernel6layer_with_weights-0/kernel/.ATTRIBUTES/VARIABLE_VALUE
YW
VARIABLE_VALUEconv2d_112/bias4layer_with_weights-0/bias/.ATTRIBUTES/VARIABLE_VALUE
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
VARIABLE_VALUEbatch_normalization_168/gamma5layer_with_weights-1/gamma/.ATTRIBUTES/VARIABLE_VALUE
fd
VARIABLE_VALUEbatch_normalization_168/beta4layer_with_weights-1/beta/.ATTRIBUTES/VARIABLE_VALUE
tr
VARIABLE_VALUE#batch_normalization_168/moving_mean;layer_with_weights-1/moving_mean/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUE'batch_normalization_168/moving_variance?layer_with_weights-1/moving_variance/.ATTRIBUTES/VARIABLE_VALUE

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
][
VARIABLE_VALUEconv2d_113/kernel6layer_with_weights-2/kernel/.ATTRIBUTES/VARIABLE_VALUE
YW
VARIABLE_VALUEconv2d_113/bias4layer_with_weights-2/bias/.ATTRIBUTES/VARIABLE_VALUE
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
VARIABLE_VALUEbatch_normalization_169/gamma5layer_with_weights-3/gamma/.ATTRIBUTES/VARIABLE_VALUE
fd
VARIABLE_VALUEbatch_normalization_169/beta4layer_with_weights-3/beta/.ATTRIBUTES/VARIABLE_VALUE
tr
VARIABLE_VALUE#batch_normalization_169/moving_mean;layer_with_weights-3/moving_mean/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUE'batch_normalization_169/moving_variance?layer_with_weights-3/moving_variance/.ATTRIBUTES/VARIABLE_VALUE

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
][
VARIABLE_VALUEconv2d_114/kernel6layer_with_weights-4/kernel/.ATTRIBUTES/VARIABLE_VALUE
YW
VARIABLE_VALUEconv2d_114/bias4layer_with_weights-4/bias/.ATTRIBUTES/VARIABLE_VALUE
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
VARIABLE_VALUEbatch_normalization_170/gamma5layer_with_weights-5/gamma/.ATTRIBUTES/VARIABLE_VALUE
fd
VARIABLE_VALUEbatch_normalization_170/beta4layer_with_weights-5/beta/.ATTRIBUTES/VARIABLE_VALUE
tr
VARIABLE_VALUE#batch_normalization_170/moving_mean;layer_with_weights-5/moving_mean/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUE'batch_normalization_170/moving_variance?layer_with_weights-5/moving_variance/.ATTRIBUTES/VARIABLE_VALUE

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
][
VARIABLE_VALUEconv2d_115/kernel6layer_with_weights-6/kernel/.ATTRIBUTES/VARIABLE_VALUE
YW
VARIABLE_VALUEconv2d_115/bias4layer_with_weights-6/bias/.ATTRIBUTES/VARIABLE_VALUE
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
VARIABLE_VALUEbatch_normalization_171/gamma5layer_with_weights-7/gamma/.ATTRIBUTES/VARIABLE_VALUE
fd
VARIABLE_VALUEbatch_normalization_171/beta4layer_with_weights-7/beta/.ATTRIBUTES/VARIABLE_VALUE
tr
VARIABLE_VALUE#batch_normalization_171/moving_mean;layer_with_weights-7/moving_mean/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUE'batch_normalization_171/moving_variance?layer_with_weights-7/moving_variance/.ATTRIBUTES/VARIABLE_VALUE

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
VARIABLE_VALUEdense_84/kernel6layer_with_weights-8/kernel/.ATTRIBUTES/VARIABLE_VALUE
WU
VARIABLE_VALUEdense_84/bias4layer_with_weights-8/bias/.ATTRIBUTES/VARIABLE_VALUE
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
VARIABLE_VALUEbatch_normalization_172/gamma5layer_with_weights-9/gamma/.ATTRIBUTES/VARIABLE_VALUE
fd
VARIABLE_VALUEbatch_normalization_172/beta4layer_with_weights-9/beta/.ATTRIBUTES/VARIABLE_VALUE
tr
VARIABLE_VALUE#batch_normalization_172/moving_mean;layer_with_weights-9/moving_mean/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUE'batch_normalization_172/moving_variance?layer_with_weights-9/moving_variance/.ATTRIBUTES/VARIABLE_VALUE
 
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
VARIABLE_VALUEdense_85/kernel7layer_with_weights-10/kernel/.ATTRIBUTES/VARIABLE_VALUE
XV
VARIABLE_VALUEdense_85/bias5layer_with_weights-10/bias/.ATTRIBUTES/VARIABLE_VALUE
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
VARIABLE_VALUEbatch_normalization_173/gamma6layer_with_weights-11/gamma/.ATTRIBUTES/VARIABLE_VALUE
ge
VARIABLE_VALUEbatch_normalization_173/beta5layer_with_weights-11/beta/.ATTRIBUTES/VARIABLE_VALUE
us
VARIABLE_VALUE#batch_normalization_173/moving_mean<layer_with_weights-11/moving_mean/.ATTRIBUTES/VARIABLE_VALUE
}{
VARIABLE_VALUE'batch_normalization_173/moving_variance@layer_with_weights-11/moving_variance/.ATTRIBUTES/VARIABLE_VALUE
 
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
VARIABLE_VALUEdense_86/kernel7layer_with_weights-12/kernel/.ATTRIBUTES/VARIABLE_VALUE
XV
VARIABLE_VALUEdense_86/bias5layer_with_weights-12/bias/.ATTRIBUTES/VARIABLE_VALUE
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
?~
VARIABLE_VALUEAdam/conv2d_112/kernel/mRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUEAdam/conv2d_112/bias/mPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_168/gamma/mQlayer_with_weights-1/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_168/beta/mPlayer_with_weights-1/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
?~
VARIABLE_VALUEAdam/conv2d_113/kernel/mRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUEAdam/conv2d_113/bias/mPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_169/gamma/mQlayer_with_weights-3/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_169/beta/mPlayer_with_weights-3/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
?~
VARIABLE_VALUEAdam/conv2d_114/kernel/mRlayer_with_weights-4/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUEAdam/conv2d_114/bias/mPlayer_with_weights-4/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_170/gamma/mQlayer_with_weights-5/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_170/beta/mPlayer_with_weights-5/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
?~
VARIABLE_VALUEAdam/conv2d_115/kernel/mRlayer_with_weights-6/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUEAdam/conv2d_115/bias/mPlayer_with_weights-6/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_171/gamma/mQlayer_with_weights-7/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_171/beta/mPlayer_with_weights-7/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
~|
VARIABLE_VALUEAdam/dense_84/kernel/mRlayer_with_weights-8/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
zx
VARIABLE_VALUEAdam/dense_84/bias/mPlayer_with_weights-8/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_172/gamma/mQlayer_with_weights-9/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_172/beta/mPlayer_with_weights-9/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/dense_85/kernel/mSlayer_with_weights-10/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/dense_85/bias/mQlayer_with_weights-10/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_173/gamma/mRlayer_with_weights-11/gamma/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_173/beta/mQlayer_with_weights-11/beta/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/dense_86/kernel/mSlayer_with_weights-12/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/dense_86/bias/mQlayer_with_weights-12/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
?~
VARIABLE_VALUEAdam/conv2d_112/kernel/vRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUEAdam/conv2d_112/bias/vPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_168/gamma/vQlayer_with_weights-1/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_168/beta/vPlayer_with_weights-1/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
?~
VARIABLE_VALUEAdam/conv2d_113/kernel/vRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUEAdam/conv2d_113/bias/vPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_169/gamma/vQlayer_with_weights-3/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_169/beta/vPlayer_with_weights-3/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
?~
VARIABLE_VALUEAdam/conv2d_114/kernel/vRlayer_with_weights-4/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUEAdam/conv2d_114/bias/vPlayer_with_weights-4/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_170/gamma/vQlayer_with_weights-5/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_170/beta/vPlayer_with_weights-5/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
?~
VARIABLE_VALUEAdam/conv2d_115/kernel/vRlayer_with_weights-6/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
|z
VARIABLE_VALUEAdam/conv2d_115/bias/vPlayer_with_weights-6/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_171/gamma/vQlayer_with_weights-7/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_171/beta/vPlayer_with_weights-7/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
~|
VARIABLE_VALUEAdam/dense_84/kernel/vRlayer_with_weights-8/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
zx
VARIABLE_VALUEAdam/dense_84/bias/vPlayer_with_weights-8/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_172/gamma/vQlayer_with_weights-9/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_172/beta/vPlayer_with_weights-9/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/dense_85/kernel/vSlayer_with_weights-10/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/dense_85/bias/vQlayer_with_weights-10/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE$Adam/batch_normalization_173/gamma/vRlayer_with_weights-11/gamma/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
??
VARIABLE_VALUE#Adam/batch_normalization_173/beta/vQlayer_with_weights-11/beta/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/dense_86/kernel/vSlayer_with_weights-12/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/dense_86/bias/vQlayer_with_weights-12/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
?
 serving_default_conv2d_112_inputPlaceholder*/
_output_shapes
:?????????@@*
dtype0*$
shape:?????????@@
?
StatefulPartitionedCallStatefulPartitionedCall serving_default_conv2d_112_inputconv2d_112/kernelconv2d_112/biasbatch_normalization_168/gammabatch_normalization_168/beta#batch_normalization_168/moving_mean'batch_normalization_168/moving_varianceconv2d_113/kernelconv2d_113/biasbatch_normalization_169/gammabatch_normalization_169/beta#batch_normalization_169/moving_mean'batch_normalization_169/moving_varianceconv2d_114/kernelconv2d_114/biasbatch_normalization_170/gammabatch_normalization_170/beta#batch_normalization_170/moving_mean'batch_normalization_170/moving_varianceconv2d_115/kernelconv2d_115/biasbatch_normalization_171/gammabatch_normalization_171/beta#batch_normalization_171/moving_mean'batch_normalization_171/moving_variancedense_84/kerneldense_84/bias'batch_normalization_172/moving_variancebatch_normalization_172/gamma#batch_normalization_172/moving_meanbatch_normalization_172/betadense_85/kerneldense_85/bias'batch_normalization_173/moving_variancebatch_normalization_173/gamma#batch_normalization_173/moving_meanbatch_normalization_173/betadense_86/kerneldense_86/bias*2
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
GPU2*0J 8? *.
f)R'
%__inference_signature_wrapper_1145638
O
saver_filenamePlaceholder*
_output_shapes
: *
dtype0*
shape: 
?'
StatefulPartitionedCall_1StatefulPartitionedCallsaver_filename%conv2d_112/kernel/Read/ReadVariableOp#conv2d_112/bias/Read/ReadVariableOp1batch_normalization_168/gamma/Read/ReadVariableOp0batch_normalization_168/beta/Read/ReadVariableOp7batch_normalization_168/moving_mean/Read/ReadVariableOp;batch_normalization_168/moving_variance/Read/ReadVariableOp%conv2d_113/kernel/Read/ReadVariableOp#conv2d_113/bias/Read/ReadVariableOp1batch_normalization_169/gamma/Read/ReadVariableOp0batch_normalization_169/beta/Read/ReadVariableOp7batch_normalization_169/moving_mean/Read/ReadVariableOp;batch_normalization_169/moving_variance/Read/ReadVariableOp%conv2d_114/kernel/Read/ReadVariableOp#conv2d_114/bias/Read/ReadVariableOp1batch_normalization_170/gamma/Read/ReadVariableOp0batch_normalization_170/beta/Read/ReadVariableOp7batch_normalization_170/moving_mean/Read/ReadVariableOp;batch_normalization_170/moving_variance/Read/ReadVariableOp%conv2d_115/kernel/Read/ReadVariableOp#conv2d_115/bias/Read/ReadVariableOp1batch_normalization_171/gamma/Read/ReadVariableOp0batch_normalization_171/beta/Read/ReadVariableOp7batch_normalization_171/moving_mean/Read/ReadVariableOp;batch_normalization_171/moving_variance/Read/ReadVariableOp#dense_84/kernel/Read/ReadVariableOp!dense_84/bias/Read/ReadVariableOp1batch_normalization_172/gamma/Read/ReadVariableOp0batch_normalization_172/beta/Read/ReadVariableOp7batch_normalization_172/moving_mean/Read/ReadVariableOp;batch_normalization_172/moving_variance/Read/ReadVariableOp#dense_85/kernel/Read/ReadVariableOp!dense_85/bias/Read/ReadVariableOp1batch_normalization_173/gamma/Read/ReadVariableOp0batch_normalization_173/beta/Read/ReadVariableOp7batch_normalization_173/moving_mean/Read/ReadVariableOp;batch_normalization_173/moving_variance/Read/ReadVariableOp#dense_86/kernel/Read/ReadVariableOp!dense_86/bias/Read/ReadVariableOpAdam/iter/Read/ReadVariableOpAdam/beta_1/Read/ReadVariableOpAdam/beta_2/Read/ReadVariableOpAdam/decay/Read/ReadVariableOp&Adam/learning_rate/Read/ReadVariableOptotal/Read/ReadVariableOpcount/Read/ReadVariableOptotal_1/Read/ReadVariableOpcount_1/Read/ReadVariableOp,Adam/conv2d_112/kernel/m/Read/ReadVariableOp*Adam/conv2d_112/bias/m/Read/ReadVariableOp8Adam/batch_normalization_168/gamma/m/Read/ReadVariableOp7Adam/batch_normalization_168/beta/m/Read/ReadVariableOp,Adam/conv2d_113/kernel/m/Read/ReadVariableOp*Adam/conv2d_113/bias/m/Read/ReadVariableOp8Adam/batch_normalization_169/gamma/m/Read/ReadVariableOp7Adam/batch_normalization_169/beta/m/Read/ReadVariableOp,Adam/conv2d_114/kernel/m/Read/ReadVariableOp*Adam/conv2d_114/bias/m/Read/ReadVariableOp8Adam/batch_normalization_170/gamma/m/Read/ReadVariableOp7Adam/batch_normalization_170/beta/m/Read/ReadVariableOp,Adam/conv2d_115/kernel/m/Read/ReadVariableOp*Adam/conv2d_115/bias/m/Read/ReadVariableOp8Adam/batch_normalization_171/gamma/m/Read/ReadVariableOp7Adam/batch_normalization_171/beta/m/Read/ReadVariableOp*Adam/dense_84/kernel/m/Read/ReadVariableOp(Adam/dense_84/bias/m/Read/ReadVariableOp8Adam/batch_normalization_172/gamma/m/Read/ReadVariableOp7Adam/batch_normalization_172/beta/m/Read/ReadVariableOp*Adam/dense_85/kernel/m/Read/ReadVariableOp(Adam/dense_85/bias/m/Read/ReadVariableOp8Adam/batch_normalization_173/gamma/m/Read/ReadVariableOp7Adam/batch_normalization_173/beta/m/Read/ReadVariableOp*Adam/dense_86/kernel/m/Read/ReadVariableOp(Adam/dense_86/bias/m/Read/ReadVariableOp,Adam/conv2d_112/kernel/v/Read/ReadVariableOp*Adam/conv2d_112/bias/v/Read/ReadVariableOp8Adam/batch_normalization_168/gamma/v/Read/ReadVariableOp7Adam/batch_normalization_168/beta/v/Read/ReadVariableOp,Adam/conv2d_113/kernel/v/Read/ReadVariableOp*Adam/conv2d_113/bias/v/Read/ReadVariableOp8Adam/batch_normalization_169/gamma/v/Read/ReadVariableOp7Adam/batch_normalization_169/beta/v/Read/ReadVariableOp,Adam/conv2d_114/kernel/v/Read/ReadVariableOp*Adam/conv2d_114/bias/v/Read/ReadVariableOp8Adam/batch_normalization_170/gamma/v/Read/ReadVariableOp7Adam/batch_normalization_170/beta/v/Read/ReadVariableOp,Adam/conv2d_115/kernel/v/Read/ReadVariableOp*Adam/conv2d_115/bias/v/Read/ReadVariableOp8Adam/batch_normalization_171/gamma/v/Read/ReadVariableOp7Adam/batch_normalization_171/beta/v/Read/ReadVariableOp*Adam/dense_84/kernel/v/Read/ReadVariableOp(Adam/dense_84/bias/v/Read/ReadVariableOp8Adam/batch_normalization_172/gamma/v/Read/ReadVariableOp7Adam/batch_normalization_172/beta/v/Read/ReadVariableOp*Adam/dense_85/kernel/v/Read/ReadVariableOp(Adam/dense_85/bias/v/Read/ReadVariableOp8Adam/batch_normalization_173/gamma/v/Read/ReadVariableOp7Adam/batch_normalization_173/beta/v/Read/ReadVariableOp*Adam/dense_86/kernel/v/Read/ReadVariableOp(Adam/dense_86/bias/v/Read/ReadVariableOpConst*p
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
GPU2*0J 8? *)
f$R"
 __inference__traced_save_1147463
?
StatefulPartitionedCall_2StatefulPartitionedCallsaver_filenameconv2d_112/kernelconv2d_112/biasbatch_normalization_168/gammabatch_normalization_168/beta#batch_normalization_168/moving_mean'batch_normalization_168/moving_varianceconv2d_113/kernelconv2d_113/biasbatch_normalization_169/gammabatch_normalization_169/beta#batch_normalization_169/moving_mean'batch_normalization_169/moving_varianceconv2d_114/kernelconv2d_114/biasbatch_normalization_170/gammabatch_normalization_170/beta#batch_normalization_170/moving_mean'batch_normalization_170/moving_varianceconv2d_115/kernelconv2d_115/biasbatch_normalization_171/gammabatch_normalization_171/beta#batch_normalization_171/moving_mean'batch_normalization_171/moving_variancedense_84/kerneldense_84/biasbatch_normalization_172/gammabatch_normalization_172/beta#batch_normalization_172/moving_mean'batch_normalization_172/moving_variancedense_85/kerneldense_85/biasbatch_normalization_173/gammabatch_normalization_173/beta#batch_normalization_173/moving_mean'batch_normalization_173/moving_variancedense_86/kerneldense_86/bias	Adam/iterAdam/beta_1Adam/beta_2
Adam/decayAdam/learning_ratetotalcounttotal_1count_1Adam/conv2d_112/kernel/mAdam/conv2d_112/bias/m$Adam/batch_normalization_168/gamma/m#Adam/batch_normalization_168/beta/mAdam/conv2d_113/kernel/mAdam/conv2d_113/bias/m$Adam/batch_normalization_169/gamma/m#Adam/batch_normalization_169/beta/mAdam/conv2d_114/kernel/mAdam/conv2d_114/bias/m$Adam/batch_normalization_170/gamma/m#Adam/batch_normalization_170/beta/mAdam/conv2d_115/kernel/mAdam/conv2d_115/bias/m$Adam/batch_normalization_171/gamma/m#Adam/batch_normalization_171/beta/mAdam/dense_84/kernel/mAdam/dense_84/bias/m$Adam/batch_normalization_172/gamma/m#Adam/batch_normalization_172/beta/mAdam/dense_85/kernel/mAdam/dense_85/bias/m$Adam/batch_normalization_173/gamma/m#Adam/batch_normalization_173/beta/mAdam/dense_86/kernel/mAdam/dense_86/bias/mAdam/conv2d_112/kernel/vAdam/conv2d_112/bias/v$Adam/batch_normalization_168/gamma/v#Adam/batch_normalization_168/beta/vAdam/conv2d_113/kernel/vAdam/conv2d_113/bias/v$Adam/batch_normalization_169/gamma/v#Adam/batch_normalization_169/beta/vAdam/conv2d_114/kernel/vAdam/conv2d_114/bias/v$Adam/batch_normalization_170/gamma/v#Adam/batch_normalization_170/beta/vAdam/conv2d_115/kernel/vAdam/conv2d_115/bias/v$Adam/batch_normalization_171/gamma/v#Adam/batch_normalization_171/beta/vAdam/dense_84/kernel/vAdam/dense_84/bias/v$Adam/batch_normalization_172/gamma/v#Adam/batch_normalization_172/beta/vAdam/dense_85/kernel/vAdam/dense_85/bias/v$Adam/batch_normalization_173/gamma/v#Adam/batch_normalization_173/beta/vAdam/dense_86/kernel/vAdam/dense_86/bias/v*o
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
GPU2*0J 8? *,
f'R%
#__inference__traced_restore_1147770??
?
c
G__inference_flatten_56_layer_call_and_return_conditional_losses_1144793

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
?
?
T__inference_batch_normalization_173_layer_call_and_return_conditional_losses_1144292

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
?
?
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_1146681

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
?
c
G__inference_flatten_57_layer_call_and_return_conditional_losses_1144911

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
?	
?
G__inference_conv2d_112_layer_call_and_return_conditional_losses_1144317

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
?
?
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_1144495

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
??
?'
"__inference__wrapped_model_1143583
conv2d_112_input;
7sequential_28_conv2d_112_conv2d_readvariableop_resource<
8sequential_28_conv2d_112_biasadd_readvariableop_resourceA
=sequential_28_batch_normalization_168_readvariableop_resourceC
?sequential_28_batch_normalization_168_readvariableop_1_resourceR
Nsequential_28_batch_normalization_168_fusedbatchnormv3_readvariableop_resourceT
Psequential_28_batch_normalization_168_fusedbatchnormv3_readvariableop_1_resource;
7sequential_28_conv2d_113_conv2d_readvariableop_resource<
8sequential_28_conv2d_113_biasadd_readvariableop_resourceA
=sequential_28_batch_normalization_169_readvariableop_resourceC
?sequential_28_batch_normalization_169_readvariableop_1_resourceR
Nsequential_28_batch_normalization_169_fusedbatchnormv3_readvariableop_resourceT
Psequential_28_batch_normalization_169_fusedbatchnormv3_readvariableop_1_resource;
7sequential_28_conv2d_114_conv2d_readvariableop_resource<
8sequential_28_conv2d_114_biasadd_readvariableop_resourceA
=sequential_28_batch_normalization_170_readvariableop_resourceC
?sequential_28_batch_normalization_170_readvariableop_1_resourceR
Nsequential_28_batch_normalization_170_fusedbatchnormv3_readvariableop_resourceT
Psequential_28_batch_normalization_170_fusedbatchnormv3_readvariableop_1_resource;
7sequential_28_conv2d_115_conv2d_readvariableop_resource<
8sequential_28_conv2d_115_biasadd_readvariableop_resourceA
=sequential_28_batch_normalization_171_readvariableop_resourceC
?sequential_28_batch_normalization_171_readvariableop_1_resourceR
Nsequential_28_batch_normalization_171_fusedbatchnormv3_readvariableop_resourceT
Psequential_28_batch_normalization_171_fusedbatchnormv3_readvariableop_1_resource9
5sequential_28_dense_84_matmul_readvariableop_resource:
6sequential_28_dense_84_biasadd_readvariableop_resourceK
Gsequential_28_batch_normalization_172_batchnorm_readvariableop_resourceO
Ksequential_28_batch_normalization_172_batchnorm_mul_readvariableop_resourceM
Isequential_28_batch_normalization_172_batchnorm_readvariableop_1_resourceM
Isequential_28_batch_normalization_172_batchnorm_readvariableop_2_resource9
5sequential_28_dense_85_matmul_readvariableop_resource:
6sequential_28_dense_85_biasadd_readvariableop_resourceK
Gsequential_28_batch_normalization_173_batchnorm_readvariableop_resourceO
Ksequential_28_batch_normalization_173_batchnorm_mul_readvariableop_resourceM
Isequential_28_batch_normalization_173_batchnorm_readvariableop_1_resourceM
Isequential_28_batch_normalization_173_batchnorm_readvariableop_2_resource9
5sequential_28_dense_86_matmul_readvariableop_resource:
6sequential_28_dense_86_biasadd_readvariableop_resource
identity??Esequential_28/batch_normalization_168/FusedBatchNormV3/ReadVariableOp?Gsequential_28/batch_normalization_168/FusedBatchNormV3/ReadVariableOp_1?4sequential_28/batch_normalization_168/ReadVariableOp?6sequential_28/batch_normalization_168/ReadVariableOp_1?Esequential_28/batch_normalization_169/FusedBatchNormV3/ReadVariableOp?Gsequential_28/batch_normalization_169/FusedBatchNormV3/ReadVariableOp_1?4sequential_28/batch_normalization_169/ReadVariableOp?6sequential_28/batch_normalization_169/ReadVariableOp_1?Esequential_28/batch_normalization_170/FusedBatchNormV3/ReadVariableOp?Gsequential_28/batch_normalization_170/FusedBatchNormV3/ReadVariableOp_1?4sequential_28/batch_normalization_170/ReadVariableOp?6sequential_28/batch_normalization_170/ReadVariableOp_1?Esequential_28/batch_normalization_171/FusedBatchNormV3/ReadVariableOp?Gsequential_28/batch_normalization_171/FusedBatchNormV3/ReadVariableOp_1?4sequential_28/batch_normalization_171/ReadVariableOp?6sequential_28/batch_normalization_171/ReadVariableOp_1?>sequential_28/batch_normalization_172/batchnorm/ReadVariableOp?@sequential_28/batch_normalization_172/batchnorm/ReadVariableOp_1?@sequential_28/batch_normalization_172/batchnorm/ReadVariableOp_2?Bsequential_28/batch_normalization_172/batchnorm/mul/ReadVariableOp?>sequential_28/batch_normalization_173/batchnorm/ReadVariableOp?@sequential_28/batch_normalization_173/batchnorm/ReadVariableOp_1?@sequential_28/batch_normalization_173/batchnorm/ReadVariableOp_2?Bsequential_28/batch_normalization_173/batchnorm/mul/ReadVariableOp?/sequential_28/conv2d_112/BiasAdd/ReadVariableOp?.sequential_28/conv2d_112/Conv2D/ReadVariableOp?/sequential_28/conv2d_113/BiasAdd/ReadVariableOp?.sequential_28/conv2d_113/Conv2D/ReadVariableOp?/sequential_28/conv2d_114/BiasAdd/ReadVariableOp?.sequential_28/conv2d_114/Conv2D/ReadVariableOp?/sequential_28/conv2d_115/BiasAdd/ReadVariableOp?.sequential_28/conv2d_115/Conv2D/ReadVariableOp?-sequential_28/dense_84/BiasAdd/ReadVariableOp?,sequential_28/dense_84/MatMul/ReadVariableOp?-sequential_28/dense_85/BiasAdd/ReadVariableOp?,sequential_28/dense_85/MatMul/ReadVariableOp?-sequential_28/dense_86/BiasAdd/ReadVariableOp?,sequential_28/dense_86/MatMul/ReadVariableOp?
.sequential_28/conv2d_112/Conv2D/ReadVariableOpReadVariableOp7sequential_28_conv2d_112_conv2d_readvariableop_resource*&
_output_shapes
:<*
dtype020
.sequential_28/conv2d_112/Conv2D/ReadVariableOp?
sequential_28/conv2d_112/Conv2DConv2Dconv2d_112_input6sequential_28/conv2d_112/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????<<<*
paddingVALID*
strides
2!
sequential_28/conv2d_112/Conv2D?
/sequential_28/conv2d_112/BiasAdd/ReadVariableOpReadVariableOp8sequential_28_conv2d_112_biasadd_readvariableop_resource*
_output_shapes
:<*
dtype021
/sequential_28/conv2d_112/BiasAdd/ReadVariableOp?
 sequential_28/conv2d_112/BiasAddBiasAdd(sequential_28/conv2d_112/Conv2D:output:07sequential_28/conv2d_112/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????<<<2"
 sequential_28/conv2d_112/BiasAdd?
!sequential_28/activation_196/ReluRelu)sequential_28/conv2d_112/BiasAdd:output:0*
T0*/
_output_shapes
:?????????<<<2#
!sequential_28/activation_196/Relu?
4sequential_28/batch_normalization_168/ReadVariableOpReadVariableOp=sequential_28_batch_normalization_168_readvariableop_resource*
_output_shapes
:<*
dtype026
4sequential_28/batch_normalization_168/ReadVariableOp?
6sequential_28/batch_normalization_168/ReadVariableOp_1ReadVariableOp?sequential_28_batch_normalization_168_readvariableop_1_resource*
_output_shapes
:<*
dtype028
6sequential_28/batch_normalization_168/ReadVariableOp_1?
Esequential_28/batch_normalization_168/FusedBatchNormV3/ReadVariableOpReadVariableOpNsequential_28_batch_normalization_168_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02G
Esequential_28/batch_normalization_168/FusedBatchNormV3/ReadVariableOp?
Gsequential_28/batch_normalization_168/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpPsequential_28_batch_normalization_168_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02I
Gsequential_28/batch_normalization_168/FusedBatchNormV3/ReadVariableOp_1?
6sequential_28/batch_normalization_168/FusedBatchNormV3FusedBatchNormV3/sequential_28/activation_196/Relu:activations:0<sequential_28/batch_normalization_168/ReadVariableOp:value:0>sequential_28/batch_normalization_168/ReadVariableOp_1:value:0Msequential_28/batch_normalization_168/FusedBatchNormV3/ReadVariableOp:value:0Osequential_28/batch_normalization_168/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????<<<:<:<:<:<:*
epsilon%o?:*
is_training( 28
6sequential_28/batch_normalization_168/FusedBatchNormV3?
.sequential_28/conv2d_113/Conv2D/ReadVariableOpReadVariableOp7sequential_28_conv2d_113_conv2d_readvariableop_resource*&
_output_shapes
:<<*
dtype020
.sequential_28/conv2d_113/Conv2D/ReadVariableOp?
sequential_28/conv2d_113/Conv2DConv2D:sequential_28/batch_normalization_168/FusedBatchNormV3:y:06sequential_28/conv2d_113/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????88<*
paddingVALID*
strides
2!
sequential_28/conv2d_113/Conv2D?
/sequential_28/conv2d_113/BiasAdd/ReadVariableOpReadVariableOp8sequential_28_conv2d_113_biasadd_readvariableop_resource*
_output_shapes
:<*
dtype021
/sequential_28/conv2d_113/BiasAdd/ReadVariableOp?
 sequential_28/conv2d_113/BiasAddBiasAdd(sequential_28/conv2d_113/Conv2D:output:07sequential_28/conv2d_113/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????88<2"
 sequential_28/conv2d_113/BiasAdd?
!sequential_28/activation_197/ReluRelu)sequential_28/conv2d_113/BiasAdd:output:0*
T0*/
_output_shapes
:?????????88<2#
!sequential_28/activation_197/Relu?
4sequential_28/batch_normalization_169/ReadVariableOpReadVariableOp=sequential_28_batch_normalization_169_readvariableop_resource*
_output_shapes
:<*
dtype026
4sequential_28/batch_normalization_169/ReadVariableOp?
6sequential_28/batch_normalization_169/ReadVariableOp_1ReadVariableOp?sequential_28_batch_normalization_169_readvariableop_1_resource*
_output_shapes
:<*
dtype028
6sequential_28/batch_normalization_169/ReadVariableOp_1?
Esequential_28/batch_normalization_169/FusedBatchNormV3/ReadVariableOpReadVariableOpNsequential_28_batch_normalization_169_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype02G
Esequential_28/batch_normalization_169/FusedBatchNormV3/ReadVariableOp?
Gsequential_28/batch_normalization_169/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpPsequential_28_batch_normalization_169_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02I
Gsequential_28/batch_normalization_169/FusedBatchNormV3/ReadVariableOp_1?
6sequential_28/batch_normalization_169/FusedBatchNormV3FusedBatchNormV3/sequential_28/activation_197/Relu:activations:0<sequential_28/batch_normalization_169/ReadVariableOp:value:0>sequential_28/batch_normalization_169/ReadVariableOp_1:value:0Msequential_28/batch_normalization_169/FusedBatchNormV3/ReadVariableOp:value:0Osequential_28/batch_normalization_169/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????88<:<:<:<:<:*
epsilon%o?:*
is_training( 28
6sequential_28/batch_normalization_169/FusedBatchNormV3?
&sequential_28/max_pooling2d_56/MaxPoolMaxPool:sequential_28/batch_normalization_169/FusedBatchNormV3:y:0*/
_output_shapes
:?????????<*
ksize
*
paddingVALID*
strides
2(
&sequential_28/max_pooling2d_56/MaxPool?
.sequential_28/conv2d_114/Conv2D/ReadVariableOpReadVariableOp7sequential_28_conv2d_114_conv2d_readvariableop_resource*&
_output_shapes
:<*
dtype020
.sequential_28/conv2d_114/Conv2D/ReadVariableOp?
sequential_28/conv2d_114/Conv2DConv2D/sequential_28/max_pooling2d_56/MaxPool:output:06sequential_28/conv2d_114/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2!
sequential_28/conv2d_114/Conv2D?
/sequential_28/conv2d_114/BiasAdd/ReadVariableOpReadVariableOp8sequential_28_conv2d_114_biasadd_readvariableop_resource*
_output_shapes
:*
dtype021
/sequential_28/conv2d_114/BiasAdd/ReadVariableOp?
 sequential_28/conv2d_114/BiasAddBiasAdd(sequential_28/conv2d_114/Conv2D:output:07sequential_28/conv2d_114/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????2"
 sequential_28/conv2d_114/BiasAdd?
!sequential_28/activation_198/ReluRelu)sequential_28/conv2d_114/BiasAdd:output:0*
T0*/
_output_shapes
:?????????2#
!sequential_28/activation_198/Relu?
4sequential_28/batch_normalization_170/ReadVariableOpReadVariableOp=sequential_28_batch_normalization_170_readvariableop_resource*
_output_shapes
:*
dtype026
4sequential_28/batch_normalization_170/ReadVariableOp?
6sequential_28/batch_normalization_170/ReadVariableOp_1ReadVariableOp?sequential_28_batch_normalization_170_readvariableop_1_resource*
_output_shapes
:*
dtype028
6sequential_28/batch_normalization_170/ReadVariableOp_1?
Esequential_28/batch_normalization_170/FusedBatchNormV3/ReadVariableOpReadVariableOpNsequential_28_batch_normalization_170_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02G
Esequential_28/batch_normalization_170/FusedBatchNormV3/ReadVariableOp?
Gsequential_28/batch_normalization_170/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpPsequential_28_batch_normalization_170_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02I
Gsequential_28/batch_normalization_170/FusedBatchNormV3/ReadVariableOp_1?
6sequential_28/batch_normalization_170/FusedBatchNormV3FusedBatchNormV3/sequential_28/activation_198/Relu:activations:0<sequential_28/batch_normalization_170/ReadVariableOp:value:0>sequential_28/batch_normalization_170/ReadVariableOp_1:value:0Msequential_28/batch_normalization_170/FusedBatchNormV3/ReadVariableOp:value:0Osequential_28/batch_normalization_170/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
is_training( 28
6sequential_28/batch_normalization_170/FusedBatchNormV3?
.sequential_28/conv2d_115/Conv2D/ReadVariableOpReadVariableOp7sequential_28_conv2d_115_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype020
.sequential_28/conv2d_115/Conv2D/ReadVariableOp?
sequential_28/conv2d_115/Conv2DConv2D:sequential_28/batch_normalization_170/FusedBatchNormV3:y:06sequential_28/conv2d_115/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2!
sequential_28/conv2d_115/Conv2D?
/sequential_28/conv2d_115/BiasAdd/ReadVariableOpReadVariableOp8sequential_28_conv2d_115_biasadd_readvariableop_resource*
_output_shapes
:*
dtype021
/sequential_28/conv2d_115/BiasAdd/ReadVariableOp?
 sequential_28/conv2d_115/BiasAddBiasAdd(sequential_28/conv2d_115/Conv2D:output:07sequential_28/conv2d_115/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????2"
 sequential_28/conv2d_115/BiasAdd?
!sequential_28/activation_199/ReluRelu)sequential_28/conv2d_115/BiasAdd:output:0*
T0*/
_output_shapes
:?????????2#
!sequential_28/activation_199/Relu?
4sequential_28/batch_normalization_171/ReadVariableOpReadVariableOp=sequential_28_batch_normalization_171_readvariableop_resource*
_output_shapes
:*
dtype026
4sequential_28/batch_normalization_171/ReadVariableOp?
6sequential_28/batch_normalization_171/ReadVariableOp_1ReadVariableOp?sequential_28_batch_normalization_171_readvariableop_1_resource*
_output_shapes
:*
dtype028
6sequential_28/batch_normalization_171/ReadVariableOp_1?
Esequential_28/batch_normalization_171/FusedBatchNormV3/ReadVariableOpReadVariableOpNsequential_28_batch_normalization_171_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype02G
Esequential_28/batch_normalization_171/FusedBatchNormV3/ReadVariableOp?
Gsequential_28/batch_normalization_171/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpPsequential_28_batch_normalization_171_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02I
Gsequential_28/batch_normalization_171/FusedBatchNormV3/ReadVariableOp_1?
6sequential_28/batch_normalization_171/FusedBatchNormV3FusedBatchNormV3/sequential_28/activation_199/Relu:activations:0<sequential_28/batch_normalization_171/ReadVariableOp:value:0>sequential_28/batch_normalization_171/ReadVariableOp_1:value:0Msequential_28/batch_normalization_171/FusedBatchNormV3/ReadVariableOp:value:0Osequential_28/batch_normalization_171/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
is_training( 28
6sequential_28/batch_normalization_171/FusedBatchNormV3?
&sequential_28/max_pooling2d_57/MaxPoolMaxPool:sequential_28/batch_normalization_171/FusedBatchNormV3:y:0*/
_output_shapes
:?????????*
ksize
*
paddingVALID*
strides
2(
&sequential_28/max_pooling2d_57/MaxPool?
!sequential_28/dropout_84/IdentityIdentity/sequential_28/max_pooling2d_57/MaxPool:output:0*
T0*/
_output_shapes
:?????????2#
!sequential_28/dropout_84/Identity?
sequential_28/flatten_56/ConstConst*
_output_shapes
:*
dtype0*
valueB"?????  2 
sequential_28/flatten_56/Const?
 sequential_28/flatten_56/ReshapeReshape*sequential_28/dropout_84/Identity:output:0'sequential_28/flatten_56/Const:output:0*
T0*(
_output_shapes
:??????????!2"
 sequential_28/flatten_56/Reshape?
,sequential_28/dense_84/MatMul/ReadVariableOpReadVariableOp5sequential_28_dense_84_matmul_readvariableop_resource* 
_output_shapes
:
?!?*
dtype02.
,sequential_28/dense_84/MatMul/ReadVariableOp?
sequential_28/dense_84/MatMulMatMul)sequential_28/flatten_56/Reshape:output:04sequential_28/dense_84/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
sequential_28/dense_84/MatMul?
-sequential_28/dense_84/BiasAdd/ReadVariableOpReadVariableOp6sequential_28_dense_84_biasadd_readvariableop_resource*
_output_shapes	
:?*
dtype02/
-sequential_28/dense_84/BiasAdd/ReadVariableOp?
sequential_28/dense_84/BiasAddBiasAdd'sequential_28/dense_84/MatMul:product:05sequential_28/dense_84/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2 
sequential_28/dense_84/BiasAdd?
!sequential_28/activation_200/ReluRelu'sequential_28/dense_84/BiasAdd:output:0*
T0*(
_output_shapes
:??????????2#
!sequential_28/activation_200/Relu?
>sequential_28/batch_normalization_172/batchnorm/ReadVariableOpReadVariableOpGsequential_28_batch_normalization_172_batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype02@
>sequential_28/batch_normalization_172/batchnorm/ReadVariableOp?
5sequential_28/batch_normalization_172/batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:27
5sequential_28/batch_normalization_172/batchnorm/add/y?
3sequential_28/batch_normalization_172/batchnorm/addAddV2Fsequential_28/batch_normalization_172/batchnorm/ReadVariableOp:value:0>sequential_28/batch_normalization_172/batchnorm/add/y:output:0*
T0*
_output_shapes	
:?25
3sequential_28/batch_normalization_172/batchnorm/add?
5sequential_28/batch_normalization_172/batchnorm/RsqrtRsqrt7sequential_28/batch_normalization_172/batchnorm/add:z:0*
T0*
_output_shapes	
:?27
5sequential_28/batch_normalization_172/batchnorm/Rsqrt?
Bsequential_28/batch_normalization_172/batchnorm/mul/ReadVariableOpReadVariableOpKsequential_28_batch_normalization_172_batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype02D
Bsequential_28/batch_normalization_172/batchnorm/mul/ReadVariableOp?
3sequential_28/batch_normalization_172/batchnorm/mulMul9sequential_28/batch_normalization_172/batchnorm/Rsqrt:y:0Jsequential_28/batch_normalization_172/batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?25
3sequential_28/batch_normalization_172/batchnorm/mul?
5sequential_28/batch_normalization_172/batchnorm/mul_1Mul/sequential_28/activation_200/Relu:activations:07sequential_28/batch_normalization_172/batchnorm/mul:z:0*
T0*(
_output_shapes
:??????????27
5sequential_28/batch_normalization_172/batchnorm/mul_1?
@sequential_28/batch_normalization_172/batchnorm/ReadVariableOp_1ReadVariableOpIsequential_28_batch_normalization_172_batchnorm_readvariableop_1_resource*
_output_shapes	
:?*
dtype02B
@sequential_28/batch_normalization_172/batchnorm/ReadVariableOp_1?
5sequential_28/batch_normalization_172/batchnorm/mul_2MulHsequential_28/batch_normalization_172/batchnorm/ReadVariableOp_1:value:07sequential_28/batch_normalization_172/batchnorm/mul:z:0*
T0*
_output_shapes	
:?27
5sequential_28/batch_normalization_172/batchnorm/mul_2?
@sequential_28/batch_normalization_172/batchnorm/ReadVariableOp_2ReadVariableOpIsequential_28_batch_normalization_172_batchnorm_readvariableop_2_resource*
_output_shapes	
:?*
dtype02B
@sequential_28/batch_normalization_172/batchnorm/ReadVariableOp_2?
3sequential_28/batch_normalization_172/batchnorm/subSubHsequential_28/batch_normalization_172/batchnorm/ReadVariableOp_2:value:09sequential_28/batch_normalization_172/batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?25
3sequential_28/batch_normalization_172/batchnorm/sub?
5sequential_28/batch_normalization_172/batchnorm/add_1AddV29sequential_28/batch_normalization_172/batchnorm/mul_1:z:07sequential_28/batch_normalization_172/batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????27
5sequential_28/batch_normalization_172/batchnorm/add_1?
!sequential_28/dropout_85/IdentityIdentity9sequential_28/batch_normalization_172/batchnorm/add_1:z:0*
T0*(
_output_shapes
:??????????2#
!sequential_28/dropout_85/Identity?
sequential_28/flatten_57/ConstConst*
_output_shapes
:*
dtype0*
valueB"????,  2 
sequential_28/flatten_57/Const?
 sequential_28/flatten_57/ReshapeReshape*sequential_28/dropout_85/Identity:output:0'sequential_28/flatten_57/Const:output:0*
T0*(
_output_shapes
:??????????2"
 sequential_28/flatten_57/Reshape?
,sequential_28/dense_85/MatMul/ReadVariableOpReadVariableOp5sequential_28_dense_85_matmul_readvariableop_resource* 
_output_shapes
:
??*
dtype02.
,sequential_28/dense_85/MatMul/ReadVariableOp?
sequential_28/dense_85/MatMulMatMul)sequential_28/flatten_57/Reshape:output:04sequential_28/dense_85/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
sequential_28/dense_85/MatMul?
-sequential_28/dense_85/BiasAdd/ReadVariableOpReadVariableOp6sequential_28_dense_85_biasadd_readvariableop_resource*
_output_shapes	
:?*
dtype02/
-sequential_28/dense_85/BiasAdd/ReadVariableOp?
sequential_28/dense_85/BiasAddBiasAdd'sequential_28/dense_85/MatMul:product:05sequential_28/dense_85/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2 
sequential_28/dense_85/BiasAdd?
!sequential_28/activation_201/ReluRelu'sequential_28/dense_85/BiasAdd:output:0*
T0*(
_output_shapes
:??????????2#
!sequential_28/activation_201/Relu?
>sequential_28/batch_normalization_173/batchnorm/ReadVariableOpReadVariableOpGsequential_28_batch_normalization_173_batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype02@
>sequential_28/batch_normalization_173/batchnorm/ReadVariableOp?
5sequential_28/batch_normalization_173/batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:27
5sequential_28/batch_normalization_173/batchnorm/add/y?
3sequential_28/batch_normalization_173/batchnorm/addAddV2Fsequential_28/batch_normalization_173/batchnorm/ReadVariableOp:value:0>sequential_28/batch_normalization_173/batchnorm/add/y:output:0*
T0*
_output_shapes	
:?25
3sequential_28/batch_normalization_173/batchnorm/add?
5sequential_28/batch_normalization_173/batchnorm/RsqrtRsqrt7sequential_28/batch_normalization_173/batchnorm/add:z:0*
T0*
_output_shapes	
:?27
5sequential_28/batch_normalization_173/batchnorm/Rsqrt?
Bsequential_28/batch_normalization_173/batchnorm/mul/ReadVariableOpReadVariableOpKsequential_28_batch_normalization_173_batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype02D
Bsequential_28/batch_normalization_173/batchnorm/mul/ReadVariableOp?
3sequential_28/batch_normalization_173/batchnorm/mulMul9sequential_28/batch_normalization_173/batchnorm/Rsqrt:y:0Jsequential_28/batch_normalization_173/batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?25
3sequential_28/batch_normalization_173/batchnorm/mul?
5sequential_28/batch_normalization_173/batchnorm/mul_1Mul/sequential_28/activation_201/Relu:activations:07sequential_28/batch_normalization_173/batchnorm/mul:z:0*
T0*(
_output_shapes
:??????????27
5sequential_28/batch_normalization_173/batchnorm/mul_1?
@sequential_28/batch_normalization_173/batchnorm/ReadVariableOp_1ReadVariableOpIsequential_28_batch_normalization_173_batchnorm_readvariableop_1_resource*
_output_shapes	
:?*
dtype02B
@sequential_28/batch_normalization_173/batchnorm/ReadVariableOp_1?
5sequential_28/batch_normalization_173/batchnorm/mul_2MulHsequential_28/batch_normalization_173/batchnorm/ReadVariableOp_1:value:07sequential_28/batch_normalization_173/batchnorm/mul:z:0*
T0*
_output_shapes	
:?27
5sequential_28/batch_normalization_173/batchnorm/mul_2?
@sequential_28/batch_normalization_173/batchnorm/ReadVariableOp_2ReadVariableOpIsequential_28_batch_normalization_173_batchnorm_readvariableop_2_resource*
_output_shapes	
:?*
dtype02B
@sequential_28/batch_normalization_173/batchnorm/ReadVariableOp_2?
3sequential_28/batch_normalization_173/batchnorm/subSubHsequential_28/batch_normalization_173/batchnorm/ReadVariableOp_2:value:09sequential_28/batch_normalization_173/batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?25
3sequential_28/batch_normalization_173/batchnorm/sub?
5sequential_28/batch_normalization_173/batchnorm/add_1AddV29sequential_28/batch_normalization_173/batchnorm/mul_1:z:07sequential_28/batch_normalization_173/batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????27
5sequential_28/batch_normalization_173/batchnorm/add_1?
!sequential_28/dropout_86/IdentityIdentity9sequential_28/batch_normalization_173/batchnorm/add_1:z:0*
T0*(
_output_shapes
:??????????2#
!sequential_28/dropout_86/Identity?
,sequential_28/dense_86/MatMul/ReadVariableOpReadVariableOp5sequential_28_dense_86_matmul_readvariableop_resource*
_output_shapes
:	?*
dtype02.
,sequential_28/dense_86/MatMul/ReadVariableOp?
sequential_28/dense_86/MatMulMatMul*sequential_28/dropout_86/Identity:output:04sequential_28/dense_86/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
sequential_28/dense_86/MatMul?
-sequential_28/dense_86/BiasAdd/ReadVariableOpReadVariableOp6sequential_28_dense_86_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02/
-sequential_28/dense_86/BiasAdd/ReadVariableOp?
sequential_28/dense_86/BiasAddBiasAdd'sequential_28/dense_86/MatMul:product:05sequential_28/dense_86/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2 
sequential_28/dense_86/BiasAdd?
$sequential_28/activation_202/SoftmaxSoftmax'sequential_28/dense_86/BiasAdd:output:0*
T0*'
_output_shapes
:?????????2&
$sequential_28/activation_202/Softmax?
IdentityIdentity.sequential_28/activation_202/Softmax:softmax:0F^sequential_28/batch_normalization_168/FusedBatchNormV3/ReadVariableOpH^sequential_28/batch_normalization_168/FusedBatchNormV3/ReadVariableOp_15^sequential_28/batch_normalization_168/ReadVariableOp7^sequential_28/batch_normalization_168/ReadVariableOp_1F^sequential_28/batch_normalization_169/FusedBatchNormV3/ReadVariableOpH^sequential_28/batch_normalization_169/FusedBatchNormV3/ReadVariableOp_15^sequential_28/batch_normalization_169/ReadVariableOp7^sequential_28/batch_normalization_169/ReadVariableOp_1F^sequential_28/batch_normalization_170/FusedBatchNormV3/ReadVariableOpH^sequential_28/batch_normalization_170/FusedBatchNormV3/ReadVariableOp_15^sequential_28/batch_normalization_170/ReadVariableOp7^sequential_28/batch_normalization_170/ReadVariableOp_1F^sequential_28/batch_normalization_171/FusedBatchNormV3/ReadVariableOpH^sequential_28/batch_normalization_171/FusedBatchNormV3/ReadVariableOp_15^sequential_28/batch_normalization_171/ReadVariableOp7^sequential_28/batch_normalization_171/ReadVariableOp_1?^sequential_28/batch_normalization_172/batchnorm/ReadVariableOpA^sequential_28/batch_normalization_172/batchnorm/ReadVariableOp_1A^sequential_28/batch_normalization_172/batchnorm/ReadVariableOp_2C^sequential_28/batch_normalization_172/batchnorm/mul/ReadVariableOp?^sequential_28/batch_normalization_173/batchnorm/ReadVariableOpA^sequential_28/batch_normalization_173/batchnorm/ReadVariableOp_1A^sequential_28/batch_normalization_173/batchnorm/ReadVariableOp_2C^sequential_28/batch_normalization_173/batchnorm/mul/ReadVariableOp0^sequential_28/conv2d_112/BiasAdd/ReadVariableOp/^sequential_28/conv2d_112/Conv2D/ReadVariableOp0^sequential_28/conv2d_113/BiasAdd/ReadVariableOp/^sequential_28/conv2d_113/Conv2D/ReadVariableOp0^sequential_28/conv2d_114/BiasAdd/ReadVariableOp/^sequential_28/conv2d_114/Conv2D/ReadVariableOp0^sequential_28/conv2d_115/BiasAdd/ReadVariableOp/^sequential_28/conv2d_115/Conv2D/ReadVariableOp.^sequential_28/dense_84/BiasAdd/ReadVariableOp-^sequential_28/dense_84/MatMul/ReadVariableOp.^sequential_28/dense_85/BiasAdd/ReadVariableOp-^sequential_28/dense_85/MatMul/ReadVariableOp.^sequential_28/dense_86/BiasAdd/ReadVariableOp-^sequential_28/dense_86/MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::2?
Esequential_28/batch_normalization_168/FusedBatchNormV3/ReadVariableOpEsequential_28/batch_normalization_168/FusedBatchNormV3/ReadVariableOp2?
Gsequential_28/batch_normalization_168/FusedBatchNormV3/ReadVariableOp_1Gsequential_28/batch_normalization_168/FusedBatchNormV3/ReadVariableOp_12l
4sequential_28/batch_normalization_168/ReadVariableOp4sequential_28/batch_normalization_168/ReadVariableOp2p
6sequential_28/batch_normalization_168/ReadVariableOp_16sequential_28/batch_normalization_168/ReadVariableOp_12?
Esequential_28/batch_normalization_169/FusedBatchNormV3/ReadVariableOpEsequential_28/batch_normalization_169/FusedBatchNormV3/ReadVariableOp2?
Gsequential_28/batch_normalization_169/FusedBatchNormV3/ReadVariableOp_1Gsequential_28/batch_normalization_169/FusedBatchNormV3/ReadVariableOp_12l
4sequential_28/batch_normalization_169/ReadVariableOp4sequential_28/batch_normalization_169/ReadVariableOp2p
6sequential_28/batch_normalization_169/ReadVariableOp_16sequential_28/batch_normalization_169/ReadVariableOp_12?
Esequential_28/batch_normalization_170/FusedBatchNormV3/ReadVariableOpEsequential_28/batch_normalization_170/FusedBatchNormV3/ReadVariableOp2?
Gsequential_28/batch_normalization_170/FusedBatchNormV3/ReadVariableOp_1Gsequential_28/batch_normalization_170/FusedBatchNormV3/ReadVariableOp_12l
4sequential_28/batch_normalization_170/ReadVariableOp4sequential_28/batch_normalization_170/ReadVariableOp2p
6sequential_28/batch_normalization_170/ReadVariableOp_16sequential_28/batch_normalization_170/ReadVariableOp_12?
Esequential_28/batch_normalization_171/FusedBatchNormV3/ReadVariableOpEsequential_28/batch_normalization_171/FusedBatchNormV3/ReadVariableOp2?
Gsequential_28/batch_normalization_171/FusedBatchNormV3/ReadVariableOp_1Gsequential_28/batch_normalization_171/FusedBatchNormV3/ReadVariableOp_12l
4sequential_28/batch_normalization_171/ReadVariableOp4sequential_28/batch_normalization_171/ReadVariableOp2p
6sequential_28/batch_normalization_171/ReadVariableOp_16sequential_28/batch_normalization_171/ReadVariableOp_12?
>sequential_28/batch_normalization_172/batchnorm/ReadVariableOp>sequential_28/batch_normalization_172/batchnorm/ReadVariableOp2?
@sequential_28/batch_normalization_172/batchnorm/ReadVariableOp_1@sequential_28/batch_normalization_172/batchnorm/ReadVariableOp_12?
@sequential_28/batch_normalization_172/batchnorm/ReadVariableOp_2@sequential_28/batch_normalization_172/batchnorm/ReadVariableOp_22?
Bsequential_28/batch_normalization_172/batchnorm/mul/ReadVariableOpBsequential_28/batch_normalization_172/batchnorm/mul/ReadVariableOp2?
>sequential_28/batch_normalization_173/batchnorm/ReadVariableOp>sequential_28/batch_normalization_173/batchnorm/ReadVariableOp2?
@sequential_28/batch_normalization_173/batchnorm/ReadVariableOp_1@sequential_28/batch_normalization_173/batchnorm/ReadVariableOp_12?
@sequential_28/batch_normalization_173/batchnorm/ReadVariableOp_2@sequential_28/batch_normalization_173/batchnorm/ReadVariableOp_22?
Bsequential_28/batch_normalization_173/batchnorm/mul/ReadVariableOpBsequential_28/batch_normalization_173/batchnorm/mul/ReadVariableOp2b
/sequential_28/conv2d_112/BiasAdd/ReadVariableOp/sequential_28/conv2d_112/BiasAdd/ReadVariableOp2`
.sequential_28/conv2d_112/Conv2D/ReadVariableOp.sequential_28/conv2d_112/Conv2D/ReadVariableOp2b
/sequential_28/conv2d_113/BiasAdd/ReadVariableOp/sequential_28/conv2d_113/BiasAdd/ReadVariableOp2`
.sequential_28/conv2d_113/Conv2D/ReadVariableOp.sequential_28/conv2d_113/Conv2D/ReadVariableOp2b
/sequential_28/conv2d_114/BiasAdd/ReadVariableOp/sequential_28/conv2d_114/BiasAdd/ReadVariableOp2`
.sequential_28/conv2d_114/Conv2D/ReadVariableOp.sequential_28/conv2d_114/Conv2D/ReadVariableOp2b
/sequential_28/conv2d_115/BiasAdd/ReadVariableOp/sequential_28/conv2d_115/BiasAdd/ReadVariableOp2`
.sequential_28/conv2d_115/Conv2D/ReadVariableOp.sequential_28/conv2d_115/Conv2D/ReadVariableOp2^
-sequential_28/dense_84/BiasAdd/ReadVariableOp-sequential_28/dense_84/BiasAdd/ReadVariableOp2\
,sequential_28/dense_84/MatMul/ReadVariableOp,sequential_28/dense_84/MatMul/ReadVariableOp2^
-sequential_28/dense_85/BiasAdd/ReadVariableOp-sequential_28/dense_85/BiasAdd/ReadVariableOp2\
,sequential_28/dense_85/MatMul/ReadVariableOp,sequential_28/dense_85/MatMul/ReadVariableOp2^
-sequential_28/dense_86/BiasAdd/ReadVariableOp-sequential_28/dense_86/BiasAdd/ReadVariableOp2\
,sequential_28/dense_86/MatMul/ReadVariableOp,sequential_28/dense_86/MatMul/ReadVariableOp:a ]
/
_output_shapes
:?????????@@
*
_user_specified_nameconv2d_112_input
?
H
,__inference_dropout_84_layer_call_fn_1146816

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
GPU2*0J 8? *P
fKRI
G__inference_dropout_84_layer_call_and_return_conditional_losses_11447742
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
?
e
G__inference_dropout_85_layer_call_and_return_conditional_losses_1144892

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
?
?
9__inference_batch_normalization_171_layer_call_fn_1146712

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_11439692
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
?	
?
G__inference_conv2d_115_layer_call_and_return_conditional_losses_1146642

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
?
H
,__inference_flatten_56_layer_call_fn_1146827

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
GPU2*0J 8? *P
fKRI
G__inference_flatten_56_layer_call_and_return_conditional_losses_11447932
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
?
f
G__inference_dropout_85_layer_call_and_return_conditional_losses_1144887

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
e
G__inference_dropout_84_layer_call_and_return_conditional_losses_1144774

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
?
?
/__inference_sequential_28_layer_call_fn_1145547
conv2d_112_input
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
StatefulPartitionedCallStatefulPartitionedCallconv2d_112_inputunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
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
GPU2*0J 8? *S
fNRL
J__inference_sequential_28_layer_call_and_return_conditional_losses_11454682
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::22
StatefulPartitionedCallStatefulPartitionedCall:a ]
/
_output_shapes
:?????????@@
*
_user_specified_nameconv2d_112_input
??
?#
J__inference_sequential_28_layer_call_and_return_conditional_losses_1145849

inputs-
)conv2d_112_conv2d_readvariableop_resource.
*conv2d_112_biasadd_readvariableop_resource3
/batch_normalization_168_readvariableop_resource5
1batch_normalization_168_readvariableop_1_resourceD
@batch_normalization_168_fusedbatchnormv3_readvariableop_resourceF
Bbatch_normalization_168_fusedbatchnormv3_readvariableop_1_resource-
)conv2d_113_conv2d_readvariableop_resource.
*conv2d_113_biasadd_readvariableop_resource3
/batch_normalization_169_readvariableop_resource5
1batch_normalization_169_readvariableop_1_resourceD
@batch_normalization_169_fusedbatchnormv3_readvariableop_resourceF
Bbatch_normalization_169_fusedbatchnormv3_readvariableop_1_resource-
)conv2d_114_conv2d_readvariableop_resource.
*conv2d_114_biasadd_readvariableop_resource3
/batch_normalization_170_readvariableop_resource5
1batch_normalization_170_readvariableop_1_resourceD
@batch_normalization_170_fusedbatchnormv3_readvariableop_resourceF
Bbatch_normalization_170_fusedbatchnormv3_readvariableop_1_resource-
)conv2d_115_conv2d_readvariableop_resource.
*conv2d_115_biasadd_readvariableop_resource3
/batch_normalization_171_readvariableop_resource5
1batch_normalization_171_readvariableop_1_resourceD
@batch_normalization_171_fusedbatchnormv3_readvariableop_resourceF
Bbatch_normalization_171_fusedbatchnormv3_readvariableop_1_resource+
'dense_84_matmul_readvariableop_resource,
(dense_84_biasadd_readvariableop_resource3
/batch_normalization_172_assignmovingavg_11457605
1batch_normalization_172_assignmovingavg_1_1145766A
=batch_normalization_172_batchnorm_mul_readvariableop_resource=
9batch_normalization_172_batchnorm_readvariableop_resource+
'dense_85_matmul_readvariableop_resource,
(dense_85_biasadd_readvariableop_resource3
/batch_normalization_173_assignmovingavg_11458095
1batch_normalization_173_assignmovingavg_1_1145815A
=batch_normalization_173_batchnorm_mul_readvariableop_resource=
9batch_normalization_173_batchnorm_readvariableop_resource+
'dense_86_matmul_readvariableop_resource,
(dense_86_biasadd_readvariableop_resource
identity??&batch_normalization_168/AssignNewValue?(batch_normalization_168/AssignNewValue_1?7batch_normalization_168/FusedBatchNormV3/ReadVariableOp?9batch_normalization_168/FusedBatchNormV3/ReadVariableOp_1?&batch_normalization_168/ReadVariableOp?(batch_normalization_168/ReadVariableOp_1?&batch_normalization_169/AssignNewValue?(batch_normalization_169/AssignNewValue_1?7batch_normalization_169/FusedBatchNormV3/ReadVariableOp?9batch_normalization_169/FusedBatchNormV3/ReadVariableOp_1?&batch_normalization_169/ReadVariableOp?(batch_normalization_169/ReadVariableOp_1?&batch_normalization_170/AssignNewValue?(batch_normalization_170/AssignNewValue_1?7batch_normalization_170/FusedBatchNormV3/ReadVariableOp?9batch_normalization_170/FusedBatchNormV3/ReadVariableOp_1?&batch_normalization_170/ReadVariableOp?(batch_normalization_170/ReadVariableOp_1?&batch_normalization_171/AssignNewValue?(batch_normalization_171/AssignNewValue_1?7batch_normalization_171/FusedBatchNormV3/ReadVariableOp?9batch_normalization_171/FusedBatchNormV3/ReadVariableOp_1?&batch_normalization_171/ReadVariableOp?(batch_normalization_171/ReadVariableOp_1?;batch_normalization_172/AssignMovingAvg/AssignSubVariableOp?6batch_normalization_172/AssignMovingAvg/ReadVariableOp?=batch_normalization_172/AssignMovingAvg_1/AssignSubVariableOp?8batch_normalization_172/AssignMovingAvg_1/ReadVariableOp?0batch_normalization_172/batchnorm/ReadVariableOp?4batch_normalization_172/batchnorm/mul/ReadVariableOp?;batch_normalization_173/AssignMovingAvg/AssignSubVariableOp?6batch_normalization_173/AssignMovingAvg/ReadVariableOp?=batch_normalization_173/AssignMovingAvg_1/AssignSubVariableOp?8batch_normalization_173/AssignMovingAvg_1/ReadVariableOp?0batch_normalization_173/batchnorm/ReadVariableOp?4batch_normalization_173/batchnorm/mul/ReadVariableOp?!conv2d_112/BiasAdd/ReadVariableOp? conv2d_112/Conv2D/ReadVariableOp?!conv2d_113/BiasAdd/ReadVariableOp? conv2d_113/Conv2D/ReadVariableOp?!conv2d_114/BiasAdd/ReadVariableOp? conv2d_114/Conv2D/ReadVariableOp?!conv2d_115/BiasAdd/ReadVariableOp? conv2d_115/Conv2D/ReadVariableOp?dense_84/BiasAdd/ReadVariableOp?dense_84/MatMul/ReadVariableOp?dense_85/BiasAdd/ReadVariableOp?dense_85/MatMul/ReadVariableOp?dense_86/BiasAdd/ReadVariableOp?dense_86/MatMul/ReadVariableOp?
 conv2d_112/Conv2D/ReadVariableOpReadVariableOp)conv2d_112_conv2d_readvariableop_resource*&
_output_shapes
:<*
dtype02"
 conv2d_112/Conv2D/ReadVariableOp?
conv2d_112/Conv2DConv2Dinputs(conv2d_112/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????<<<*
paddingVALID*
strides
2
conv2d_112/Conv2D?
!conv2d_112/BiasAdd/ReadVariableOpReadVariableOp*conv2d_112_biasadd_readvariableop_resource*
_output_shapes
:<*
dtype02#
!conv2d_112/BiasAdd/ReadVariableOp?
conv2d_112/BiasAddBiasAddconv2d_112/Conv2D:output:0)conv2d_112/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????<<<2
conv2d_112/BiasAdd?
activation_196/ReluReluconv2d_112/BiasAdd:output:0*
T0*/
_output_shapes
:?????????<<<2
activation_196/Relu?
&batch_normalization_168/ReadVariableOpReadVariableOp/batch_normalization_168_readvariableop_resource*
_output_shapes
:<*
dtype02(
&batch_normalization_168/ReadVariableOp?
(batch_normalization_168/ReadVariableOp_1ReadVariableOp1batch_normalization_168_readvariableop_1_resource*
_output_shapes
:<*
dtype02*
(batch_normalization_168/ReadVariableOp_1?
7batch_normalization_168/FusedBatchNormV3/ReadVariableOpReadVariableOp@batch_normalization_168_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype029
7batch_normalization_168/FusedBatchNormV3/ReadVariableOp?
9batch_normalization_168/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpBbatch_normalization_168_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02;
9batch_normalization_168/FusedBatchNormV3/ReadVariableOp_1?
(batch_normalization_168/FusedBatchNormV3FusedBatchNormV3!activation_196/Relu:activations:0.batch_normalization_168/ReadVariableOp:value:00batch_normalization_168/ReadVariableOp_1:value:0?batch_normalization_168/FusedBatchNormV3/ReadVariableOp:value:0Abatch_normalization_168/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????<<<:<:<:<:<:*
epsilon%o?:*
exponential_avg_factor%
?#<2*
(batch_normalization_168/FusedBatchNormV3?
&batch_normalization_168/AssignNewValueAssignVariableOp@batch_normalization_168_fusedbatchnormv3_readvariableop_resource5batch_normalization_168/FusedBatchNormV3:batch_mean:08^batch_normalization_168/FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*S
_classI
GEloc:@batch_normalization_168/FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02(
&batch_normalization_168/AssignNewValue?
(batch_normalization_168/AssignNewValue_1AssignVariableOpBbatch_normalization_168_fusedbatchnormv3_readvariableop_1_resource9batch_normalization_168/FusedBatchNormV3:batch_variance:0:^batch_normalization_168/FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*U
_classK
IGloc:@batch_normalization_168/FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02*
(batch_normalization_168/AssignNewValue_1?
 conv2d_113/Conv2D/ReadVariableOpReadVariableOp)conv2d_113_conv2d_readvariableop_resource*&
_output_shapes
:<<*
dtype02"
 conv2d_113/Conv2D/ReadVariableOp?
conv2d_113/Conv2DConv2D,batch_normalization_168/FusedBatchNormV3:y:0(conv2d_113/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????88<*
paddingVALID*
strides
2
conv2d_113/Conv2D?
!conv2d_113/BiasAdd/ReadVariableOpReadVariableOp*conv2d_113_biasadd_readvariableop_resource*
_output_shapes
:<*
dtype02#
!conv2d_113/BiasAdd/ReadVariableOp?
conv2d_113/BiasAddBiasAddconv2d_113/Conv2D:output:0)conv2d_113/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????88<2
conv2d_113/BiasAdd?
activation_197/ReluReluconv2d_113/BiasAdd:output:0*
T0*/
_output_shapes
:?????????88<2
activation_197/Relu?
&batch_normalization_169/ReadVariableOpReadVariableOp/batch_normalization_169_readvariableop_resource*
_output_shapes
:<*
dtype02(
&batch_normalization_169/ReadVariableOp?
(batch_normalization_169/ReadVariableOp_1ReadVariableOp1batch_normalization_169_readvariableop_1_resource*
_output_shapes
:<*
dtype02*
(batch_normalization_169/ReadVariableOp_1?
7batch_normalization_169/FusedBatchNormV3/ReadVariableOpReadVariableOp@batch_normalization_169_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype029
7batch_normalization_169/FusedBatchNormV3/ReadVariableOp?
9batch_normalization_169/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpBbatch_normalization_169_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02;
9batch_normalization_169/FusedBatchNormV3/ReadVariableOp_1?
(batch_normalization_169/FusedBatchNormV3FusedBatchNormV3!activation_197/Relu:activations:0.batch_normalization_169/ReadVariableOp:value:00batch_normalization_169/ReadVariableOp_1:value:0?batch_normalization_169/FusedBatchNormV3/ReadVariableOp:value:0Abatch_normalization_169/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????88<:<:<:<:<:*
epsilon%o?:*
exponential_avg_factor%
?#<2*
(batch_normalization_169/FusedBatchNormV3?
&batch_normalization_169/AssignNewValueAssignVariableOp@batch_normalization_169_fusedbatchnormv3_readvariableop_resource5batch_normalization_169/FusedBatchNormV3:batch_mean:08^batch_normalization_169/FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*S
_classI
GEloc:@batch_normalization_169/FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02(
&batch_normalization_169/AssignNewValue?
(batch_normalization_169/AssignNewValue_1AssignVariableOpBbatch_normalization_169_fusedbatchnormv3_readvariableop_1_resource9batch_normalization_169/FusedBatchNormV3:batch_variance:0:^batch_normalization_169/FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*U
_classK
IGloc:@batch_normalization_169/FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02*
(batch_normalization_169/AssignNewValue_1?
max_pooling2d_56/MaxPoolMaxPool,batch_normalization_169/FusedBatchNormV3:y:0*/
_output_shapes
:?????????<*
ksize
*
paddingVALID*
strides
2
max_pooling2d_56/MaxPool?
 conv2d_114/Conv2D/ReadVariableOpReadVariableOp)conv2d_114_conv2d_readvariableop_resource*&
_output_shapes
:<*
dtype02"
 conv2d_114/Conv2D/ReadVariableOp?
conv2d_114/Conv2DConv2D!max_pooling2d_56/MaxPool:output:0(conv2d_114/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2
conv2d_114/Conv2D?
!conv2d_114/BiasAdd/ReadVariableOpReadVariableOp*conv2d_114_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02#
!conv2d_114/BiasAdd/ReadVariableOp?
conv2d_114/BiasAddBiasAddconv2d_114/Conv2D:output:0)conv2d_114/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????2
conv2d_114/BiasAdd?
activation_198/ReluReluconv2d_114/BiasAdd:output:0*
T0*/
_output_shapes
:?????????2
activation_198/Relu?
&batch_normalization_170/ReadVariableOpReadVariableOp/batch_normalization_170_readvariableop_resource*
_output_shapes
:*
dtype02(
&batch_normalization_170/ReadVariableOp?
(batch_normalization_170/ReadVariableOp_1ReadVariableOp1batch_normalization_170_readvariableop_1_resource*
_output_shapes
:*
dtype02*
(batch_normalization_170/ReadVariableOp_1?
7batch_normalization_170/FusedBatchNormV3/ReadVariableOpReadVariableOp@batch_normalization_170_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype029
7batch_normalization_170/FusedBatchNormV3/ReadVariableOp?
9batch_normalization_170/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpBbatch_normalization_170_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02;
9batch_normalization_170/FusedBatchNormV3/ReadVariableOp_1?
(batch_normalization_170/FusedBatchNormV3FusedBatchNormV3!activation_198/Relu:activations:0.batch_normalization_170/ReadVariableOp:value:00batch_normalization_170/ReadVariableOp_1:value:0?batch_normalization_170/FusedBatchNormV3/ReadVariableOp:value:0Abatch_normalization_170/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
exponential_avg_factor%
?#<2*
(batch_normalization_170/FusedBatchNormV3?
&batch_normalization_170/AssignNewValueAssignVariableOp@batch_normalization_170_fusedbatchnormv3_readvariableop_resource5batch_normalization_170/FusedBatchNormV3:batch_mean:08^batch_normalization_170/FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*S
_classI
GEloc:@batch_normalization_170/FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02(
&batch_normalization_170/AssignNewValue?
(batch_normalization_170/AssignNewValue_1AssignVariableOpBbatch_normalization_170_fusedbatchnormv3_readvariableop_1_resource9batch_normalization_170/FusedBatchNormV3:batch_variance:0:^batch_normalization_170/FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*U
_classK
IGloc:@batch_normalization_170/FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02*
(batch_normalization_170/AssignNewValue_1?
 conv2d_115/Conv2D/ReadVariableOpReadVariableOp)conv2d_115_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02"
 conv2d_115/Conv2D/ReadVariableOp?
conv2d_115/Conv2DConv2D,batch_normalization_170/FusedBatchNormV3:y:0(conv2d_115/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2
conv2d_115/Conv2D?
!conv2d_115/BiasAdd/ReadVariableOpReadVariableOp*conv2d_115_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02#
!conv2d_115/BiasAdd/ReadVariableOp?
conv2d_115/BiasAddBiasAddconv2d_115/Conv2D:output:0)conv2d_115/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????2
conv2d_115/BiasAdd?
activation_199/ReluReluconv2d_115/BiasAdd:output:0*
T0*/
_output_shapes
:?????????2
activation_199/Relu?
&batch_normalization_171/ReadVariableOpReadVariableOp/batch_normalization_171_readvariableop_resource*
_output_shapes
:*
dtype02(
&batch_normalization_171/ReadVariableOp?
(batch_normalization_171/ReadVariableOp_1ReadVariableOp1batch_normalization_171_readvariableop_1_resource*
_output_shapes
:*
dtype02*
(batch_normalization_171/ReadVariableOp_1?
7batch_normalization_171/FusedBatchNormV3/ReadVariableOpReadVariableOp@batch_normalization_171_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype029
7batch_normalization_171/FusedBatchNormV3/ReadVariableOp?
9batch_normalization_171/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpBbatch_normalization_171_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02;
9batch_normalization_171/FusedBatchNormV3/ReadVariableOp_1?
(batch_normalization_171/FusedBatchNormV3FusedBatchNormV3!activation_199/Relu:activations:0.batch_normalization_171/ReadVariableOp:value:00batch_normalization_171/ReadVariableOp_1:value:0?batch_normalization_171/FusedBatchNormV3/ReadVariableOp:value:0Abatch_normalization_171/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
exponential_avg_factor%
?#<2*
(batch_normalization_171/FusedBatchNormV3?
&batch_normalization_171/AssignNewValueAssignVariableOp@batch_normalization_171_fusedbatchnormv3_readvariableop_resource5batch_normalization_171/FusedBatchNormV3:batch_mean:08^batch_normalization_171/FusedBatchNormV3/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*S
_classI
GEloc:@batch_normalization_171/FusedBatchNormV3/ReadVariableOp/resource*
_output_shapes
 *
dtype02(
&batch_normalization_171/AssignNewValue?
(batch_normalization_171/AssignNewValue_1AssignVariableOpBbatch_normalization_171_fusedbatchnormv3_readvariableop_1_resource9batch_normalization_171/FusedBatchNormV3:batch_variance:0:^batch_normalization_171/FusedBatchNormV3/ReadVariableOp_1",/job:localhost/replica:0/task:0/device:GPU:0*U
_classK
IGloc:@batch_normalization_171/FusedBatchNormV3/ReadVariableOp_1/resource*
_output_shapes
 *
dtype02*
(batch_normalization_171/AssignNewValue_1?
max_pooling2d_57/MaxPoolMaxPool,batch_normalization_171/FusedBatchNormV3:y:0*/
_output_shapes
:?????????*
ksize
*
paddingVALID*
strides
2
max_pooling2d_57/MaxPooly
dropout_84/dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *UU??2
dropout_84/dropout/Const?
dropout_84/dropout/MulMul!max_pooling2d_57/MaxPool:output:0!dropout_84/dropout/Const:output:0*
T0*/
_output_shapes
:?????????2
dropout_84/dropout/Mul?
dropout_84/dropout/ShapeShape!max_pooling2d_57/MaxPool:output:0*
T0*
_output_shapes
:2
dropout_84/dropout/Shape?
/dropout_84/dropout/random_uniform/RandomUniformRandomUniform!dropout_84/dropout/Shape:output:0*
T0*/
_output_shapes
:?????????*
dtype021
/dropout_84/dropout/random_uniform/RandomUniform?
!dropout_84/dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *???>2#
!dropout_84/dropout/GreaterEqual/y?
dropout_84/dropout/GreaterEqualGreaterEqual8dropout_84/dropout/random_uniform/RandomUniform:output:0*dropout_84/dropout/GreaterEqual/y:output:0*
T0*/
_output_shapes
:?????????2!
dropout_84/dropout/GreaterEqual?
dropout_84/dropout/CastCast#dropout_84/dropout/GreaterEqual:z:0*

DstT0*

SrcT0
*/
_output_shapes
:?????????2
dropout_84/dropout/Cast?
dropout_84/dropout/Mul_1Muldropout_84/dropout/Mul:z:0dropout_84/dropout/Cast:y:0*
T0*/
_output_shapes
:?????????2
dropout_84/dropout/Mul_1u
flatten_56/ConstConst*
_output_shapes
:*
dtype0*
valueB"?????  2
flatten_56/Const?
flatten_56/ReshapeReshapedropout_84/dropout/Mul_1:z:0flatten_56/Const:output:0*
T0*(
_output_shapes
:??????????!2
flatten_56/Reshape?
dense_84/MatMul/ReadVariableOpReadVariableOp'dense_84_matmul_readvariableop_resource* 
_output_shapes
:
?!?*
dtype02 
dense_84/MatMul/ReadVariableOp?
dense_84/MatMulMatMulflatten_56/Reshape:output:0&dense_84/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
dense_84/MatMul?
dense_84/BiasAdd/ReadVariableOpReadVariableOp(dense_84_biasadd_readvariableop_resource*
_output_shapes	
:?*
dtype02!
dense_84/BiasAdd/ReadVariableOp?
dense_84/BiasAddBiasAdddense_84/MatMul:product:0'dense_84/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
dense_84/BiasAdd?
activation_200/ReluReludense_84/BiasAdd:output:0*
T0*(
_output_shapes
:??????????2
activation_200/Relu?
6batch_normalization_172/moments/mean/reduction_indicesConst*
_output_shapes
:*
dtype0*
valueB: 28
6batch_normalization_172/moments/mean/reduction_indices?
$batch_normalization_172/moments/meanMean!activation_200/Relu:activations:0?batch_normalization_172/moments/mean/reduction_indices:output:0*
T0*
_output_shapes
:	?*
	keep_dims(2&
$batch_normalization_172/moments/mean?
,batch_normalization_172/moments/StopGradientStopGradient-batch_normalization_172/moments/mean:output:0*
T0*
_output_shapes
:	?2.
,batch_normalization_172/moments/StopGradient?
1batch_normalization_172/moments/SquaredDifferenceSquaredDifference!activation_200/Relu:activations:05batch_normalization_172/moments/StopGradient:output:0*
T0*(
_output_shapes
:??????????23
1batch_normalization_172/moments/SquaredDifference?
:batch_normalization_172/moments/variance/reduction_indicesConst*
_output_shapes
:*
dtype0*
valueB: 2<
:batch_normalization_172/moments/variance/reduction_indices?
(batch_normalization_172/moments/varianceMean5batch_normalization_172/moments/SquaredDifference:z:0Cbatch_normalization_172/moments/variance/reduction_indices:output:0*
T0*
_output_shapes
:	?*
	keep_dims(2*
(batch_normalization_172/moments/variance?
'batch_normalization_172/moments/SqueezeSqueeze-batch_normalization_172/moments/mean:output:0*
T0*
_output_shapes	
:?*
squeeze_dims
 2)
'batch_normalization_172/moments/Squeeze?
)batch_normalization_172/moments/Squeeze_1Squeeze1batch_normalization_172/moments/variance:output:0*
T0*
_output_shapes	
:?*
squeeze_dims
 2+
)batch_normalization_172/moments/Squeeze_1?
-batch_normalization_172/AssignMovingAvg/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*B
_class8
64loc:@batch_normalization_172/AssignMovingAvg/1145760*
_output_shapes
: *
dtype0*
valueB
 *
?#<2/
-batch_normalization_172/AssignMovingAvg/decay?
6batch_normalization_172/AssignMovingAvg/ReadVariableOpReadVariableOp/batch_normalization_172_assignmovingavg_1145760*
_output_shapes	
:?*
dtype028
6batch_normalization_172/AssignMovingAvg/ReadVariableOp?
+batch_normalization_172/AssignMovingAvg/subSub>batch_normalization_172/AssignMovingAvg/ReadVariableOp:value:00batch_normalization_172/moments/Squeeze:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*B
_class8
64loc:@batch_normalization_172/AssignMovingAvg/1145760*
_output_shapes	
:?2-
+batch_normalization_172/AssignMovingAvg/sub?
+batch_normalization_172/AssignMovingAvg/mulMul/batch_normalization_172/AssignMovingAvg/sub:z:06batch_normalization_172/AssignMovingAvg/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*B
_class8
64loc:@batch_normalization_172/AssignMovingAvg/1145760*
_output_shapes	
:?2-
+batch_normalization_172/AssignMovingAvg/mul?
;batch_normalization_172/AssignMovingAvg/AssignSubVariableOpAssignSubVariableOp/batch_normalization_172_assignmovingavg_1145760/batch_normalization_172/AssignMovingAvg/mul:z:07^batch_normalization_172/AssignMovingAvg/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*B
_class8
64loc:@batch_normalization_172/AssignMovingAvg/1145760*
_output_shapes
 *
dtype02=
;batch_normalization_172/AssignMovingAvg/AssignSubVariableOp?
/batch_normalization_172/AssignMovingAvg_1/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*D
_class:
86loc:@batch_normalization_172/AssignMovingAvg_1/1145766*
_output_shapes
: *
dtype0*
valueB
 *
?#<21
/batch_normalization_172/AssignMovingAvg_1/decay?
8batch_normalization_172/AssignMovingAvg_1/ReadVariableOpReadVariableOp1batch_normalization_172_assignmovingavg_1_1145766*
_output_shapes	
:?*
dtype02:
8batch_normalization_172/AssignMovingAvg_1/ReadVariableOp?
-batch_normalization_172/AssignMovingAvg_1/subSub@batch_normalization_172/AssignMovingAvg_1/ReadVariableOp:value:02batch_normalization_172/moments/Squeeze_1:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*D
_class:
86loc:@batch_normalization_172/AssignMovingAvg_1/1145766*
_output_shapes	
:?2/
-batch_normalization_172/AssignMovingAvg_1/sub?
-batch_normalization_172/AssignMovingAvg_1/mulMul1batch_normalization_172/AssignMovingAvg_1/sub:z:08batch_normalization_172/AssignMovingAvg_1/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*D
_class:
86loc:@batch_normalization_172/AssignMovingAvg_1/1145766*
_output_shapes	
:?2/
-batch_normalization_172/AssignMovingAvg_1/mul?
=batch_normalization_172/AssignMovingAvg_1/AssignSubVariableOpAssignSubVariableOp1batch_normalization_172_assignmovingavg_1_11457661batch_normalization_172/AssignMovingAvg_1/mul:z:09^batch_normalization_172/AssignMovingAvg_1/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*D
_class:
86loc:@batch_normalization_172/AssignMovingAvg_1/1145766*
_output_shapes
 *
dtype02?
=batch_normalization_172/AssignMovingAvg_1/AssignSubVariableOp?
'batch_normalization_172/batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:2)
'batch_normalization_172/batchnorm/add/y?
%batch_normalization_172/batchnorm/addAddV22batch_normalization_172/moments/Squeeze_1:output:00batch_normalization_172/batchnorm/add/y:output:0*
T0*
_output_shapes	
:?2'
%batch_normalization_172/batchnorm/add?
'batch_normalization_172/batchnorm/RsqrtRsqrt)batch_normalization_172/batchnorm/add:z:0*
T0*
_output_shapes	
:?2)
'batch_normalization_172/batchnorm/Rsqrt?
4batch_normalization_172/batchnorm/mul/ReadVariableOpReadVariableOp=batch_normalization_172_batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype026
4batch_normalization_172/batchnorm/mul/ReadVariableOp?
%batch_normalization_172/batchnorm/mulMul+batch_normalization_172/batchnorm/Rsqrt:y:0<batch_normalization_172/batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?2'
%batch_normalization_172/batchnorm/mul?
'batch_normalization_172/batchnorm/mul_1Mul!activation_200/Relu:activations:0)batch_normalization_172/batchnorm/mul:z:0*
T0*(
_output_shapes
:??????????2)
'batch_normalization_172/batchnorm/mul_1?
'batch_normalization_172/batchnorm/mul_2Mul0batch_normalization_172/moments/Squeeze:output:0)batch_normalization_172/batchnorm/mul:z:0*
T0*
_output_shapes	
:?2)
'batch_normalization_172/batchnorm/mul_2?
0batch_normalization_172/batchnorm/ReadVariableOpReadVariableOp9batch_normalization_172_batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype022
0batch_normalization_172/batchnorm/ReadVariableOp?
%batch_normalization_172/batchnorm/subSub8batch_normalization_172/batchnorm/ReadVariableOp:value:0+batch_normalization_172/batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?2'
%batch_normalization_172/batchnorm/sub?
'batch_normalization_172/batchnorm/add_1AddV2+batch_normalization_172/batchnorm/mul_1:z:0)batch_normalization_172/batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????2)
'batch_normalization_172/batchnorm/add_1y
dropout_85/dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *UU??2
dropout_85/dropout/Const?
dropout_85/dropout/MulMul+batch_normalization_172/batchnorm/add_1:z:0!dropout_85/dropout/Const:output:0*
T0*(
_output_shapes
:??????????2
dropout_85/dropout/Mul?
dropout_85/dropout/ShapeShape+batch_normalization_172/batchnorm/add_1:z:0*
T0*
_output_shapes
:2
dropout_85/dropout/Shape?
/dropout_85/dropout/random_uniform/RandomUniformRandomUniform!dropout_85/dropout/Shape:output:0*
T0*(
_output_shapes
:??????????*
dtype021
/dropout_85/dropout/random_uniform/RandomUniform?
!dropout_85/dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *???>2#
!dropout_85/dropout/GreaterEqual/y?
dropout_85/dropout/GreaterEqualGreaterEqual8dropout_85/dropout/random_uniform/RandomUniform:output:0*dropout_85/dropout/GreaterEqual/y:output:0*
T0*(
_output_shapes
:??????????2!
dropout_85/dropout/GreaterEqual?
dropout_85/dropout/CastCast#dropout_85/dropout/GreaterEqual:z:0*

DstT0*

SrcT0
*(
_output_shapes
:??????????2
dropout_85/dropout/Cast?
dropout_85/dropout/Mul_1Muldropout_85/dropout/Mul:z:0dropout_85/dropout/Cast:y:0*
T0*(
_output_shapes
:??????????2
dropout_85/dropout/Mul_1u
flatten_57/ConstConst*
_output_shapes
:*
dtype0*
valueB"????,  2
flatten_57/Const?
flatten_57/ReshapeReshapedropout_85/dropout/Mul_1:z:0flatten_57/Const:output:0*
T0*(
_output_shapes
:??????????2
flatten_57/Reshape?
dense_85/MatMul/ReadVariableOpReadVariableOp'dense_85_matmul_readvariableop_resource* 
_output_shapes
:
??*
dtype02 
dense_85/MatMul/ReadVariableOp?
dense_85/MatMulMatMulflatten_57/Reshape:output:0&dense_85/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
dense_85/MatMul?
dense_85/BiasAdd/ReadVariableOpReadVariableOp(dense_85_biasadd_readvariableop_resource*
_output_shapes	
:?*
dtype02!
dense_85/BiasAdd/ReadVariableOp?
dense_85/BiasAddBiasAdddense_85/MatMul:product:0'dense_85/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
dense_85/BiasAdd?
activation_201/ReluReludense_85/BiasAdd:output:0*
T0*(
_output_shapes
:??????????2
activation_201/Relu?
6batch_normalization_173/moments/mean/reduction_indicesConst*
_output_shapes
:*
dtype0*
valueB: 28
6batch_normalization_173/moments/mean/reduction_indices?
$batch_normalization_173/moments/meanMean!activation_201/Relu:activations:0?batch_normalization_173/moments/mean/reduction_indices:output:0*
T0*
_output_shapes
:	?*
	keep_dims(2&
$batch_normalization_173/moments/mean?
,batch_normalization_173/moments/StopGradientStopGradient-batch_normalization_173/moments/mean:output:0*
T0*
_output_shapes
:	?2.
,batch_normalization_173/moments/StopGradient?
1batch_normalization_173/moments/SquaredDifferenceSquaredDifference!activation_201/Relu:activations:05batch_normalization_173/moments/StopGradient:output:0*
T0*(
_output_shapes
:??????????23
1batch_normalization_173/moments/SquaredDifference?
:batch_normalization_173/moments/variance/reduction_indicesConst*
_output_shapes
:*
dtype0*
valueB: 2<
:batch_normalization_173/moments/variance/reduction_indices?
(batch_normalization_173/moments/varianceMean5batch_normalization_173/moments/SquaredDifference:z:0Cbatch_normalization_173/moments/variance/reduction_indices:output:0*
T0*
_output_shapes
:	?*
	keep_dims(2*
(batch_normalization_173/moments/variance?
'batch_normalization_173/moments/SqueezeSqueeze-batch_normalization_173/moments/mean:output:0*
T0*
_output_shapes	
:?*
squeeze_dims
 2)
'batch_normalization_173/moments/Squeeze?
)batch_normalization_173/moments/Squeeze_1Squeeze1batch_normalization_173/moments/variance:output:0*
T0*
_output_shapes	
:?*
squeeze_dims
 2+
)batch_normalization_173/moments/Squeeze_1?
-batch_normalization_173/AssignMovingAvg/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*B
_class8
64loc:@batch_normalization_173/AssignMovingAvg/1145809*
_output_shapes
: *
dtype0*
valueB
 *
?#<2/
-batch_normalization_173/AssignMovingAvg/decay?
6batch_normalization_173/AssignMovingAvg/ReadVariableOpReadVariableOp/batch_normalization_173_assignmovingavg_1145809*
_output_shapes	
:?*
dtype028
6batch_normalization_173/AssignMovingAvg/ReadVariableOp?
+batch_normalization_173/AssignMovingAvg/subSub>batch_normalization_173/AssignMovingAvg/ReadVariableOp:value:00batch_normalization_173/moments/Squeeze:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*B
_class8
64loc:@batch_normalization_173/AssignMovingAvg/1145809*
_output_shapes	
:?2-
+batch_normalization_173/AssignMovingAvg/sub?
+batch_normalization_173/AssignMovingAvg/mulMul/batch_normalization_173/AssignMovingAvg/sub:z:06batch_normalization_173/AssignMovingAvg/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*B
_class8
64loc:@batch_normalization_173/AssignMovingAvg/1145809*
_output_shapes	
:?2-
+batch_normalization_173/AssignMovingAvg/mul?
;batch_normalization_173/AssignMovingAvg/AssignSubVariableOpAssignSubVariableOp/batch_normalization_173_assignmovingavg_1145809/batch_normalization_173/AssignMovingAvg/mul:z:07^batch_normalization_173/AssignMovingAvg/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*B
_class8
64loc:@batch_normalization_173/AssignMovingAvg/1145809*
_output_shapes
 *
dtype02=
;batch_normalization_173/AssignMovingAvg/AssignSubVariableOp?
/batch_normalization_173/AssignMovingAvg_1/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*D
_class:
86loc:@batch_normalization_173/AssignMovingAvg_1/1145815*
_output_shapes
: *
dtype0*
valueB
 *
?#<21
/batch_normalization_173/AssignMovingAvg_1/decay?
8batch_normalization_173/AssignMovingAvg_1/ReadVariableOpReadVariableOp1batch_normalization_173_assignmovingavg_1_1145815*
_output_shapes	
:?*
dtype02:
8batch_normalization_173/AssignMovingAvg_1/ReadVariableOp?
-batch_normalization_173/AssignMovingAvg_1/subSub@batch_normalization_173/AssignMovingAvg_1/ReadVariableOp:value:02batch_normalization_173/moments/Squeeze_1:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*D
_class:
86loc:@batch_normalization_173/AssignMovingAvg_1/1145815*
_output_shapes	
:?2/
-batch_normalization_173/AssignMovingAvg_1/sub?
-batch_normalization_173/AssignMovingAvg_1/mulMul1batch_normalization_173/AssignMovingAvg_1/sub:z:08batch_normalization_173/AssignMovingAvg_1/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*D
_class:
86loc:@batch_normalization_173/AssignMovingAvg_1/1145815*
_output_shapes	
:?2/
-batch_normalization_173/AssignMovingAvg_1/mul?
=batch_normalization_173/AssignMovingAvg_1/AssignSubVariableOpAssignSubVariableOp1batch_normalization_173_assignmovingavg_1_11458151batch_normalization_173/AssignMovingAvg_1/mul:z:09^batch_normalization_173/AssignMovingAvg_1/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*D
_class:
86loc:@batch_normalization_173/AssignMovingAvg_1/1145815*
_output_shapes
 *
dtype02?
=batch_normalization_173/AssignMovingAvg_1/AssignSubVariableOp?
'batch_normalization_173/batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:2)
'batch_normalization_173/batchnorm/add/y?
%batch_normalization_173/batchnorm/addAddV22batch_normalization_173/moments/Squeeze_1:output:00batch_normalization_173/batchnorm/add/y:output:0*
T0*
_output_shapes	
:?2'
%batch_normalization_173/batchnorm/add?
'batch_normalization_173/batchnorm/RsqrtRsqrt)batch_normalization_173/batchnorm/add:z:0*
T0*
_output_shapes	
:?2)
'batch_normalization_173/batchnorm/Rsqrt?
4batch_normalization_173/batchnorm/mul/ReadVariableOpReadVariableOp=batch_normalization_173_batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype026
4batch_normalization_173/batchnorm/mul/ReadVariableOp?
%batch_normalization_173/batchnorm/mulMul+batch_normalization_173/batchnorm/Rsqrt:y:0<batch_normalization_173/batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?2'
%batch_normalization_173/batchnorm/mul?
'batch_normalization_173/batchnorm/mul_1Mul!activation_201/Relu:activations:0)batch_normalization_173/batchnorm/mul:z:0*
T0*(
_output_shapes
:??????????2)
'batch_normalization_173/batchnorm/mul_1?
'batch_normalization_173/batchnorm/mul_2Mul0batch_normalization_173/moments/Squeeze:output:0)batch_normalization_173/batchnorm/mul:z:0*
T0*
_output_shapes	
:?2)
'batch_normalization_173/batchnorm/mul_2?
0batch_normalization_173/batchnorm/ReadVariableOpReadVariableOp9batch_normalization_173_batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype022
0batch_normalization_173/batchnorm/ReadVariableOp?
%batch_normalization_173/batchnorm/subSub8batch_normalization_173/batchnorm/ReadVariableOp:value:0+batch_normalization_173/batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?2'
%batch_normalization_173/batchnorm/sub?
'batch_normalization_173/batchnorm/add_1AddV2+batch_normalization_173/batchnorm/mul_1:z:0)batch_normalization_173/batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????2)
'batch_normalization_173/batchnorm/add_1y
dropout_86/dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *UU??2
dropout_86/dropout/Const?
dropout_86/dropout/MulMul+batch_normalization_173/batchnorm/add_1:z:0!dropout_86/dropout/Const:output:0*
T0*(
_output_shapes
:??????????2
dropout_86/dropout/Mul?
dropout_86/dropout/ShapeShape+batch_normalization_173/batchnorm/add_1:z:0*
T0*
_output_shapes
:2
dropout_86/dropout/Shape?
/dropout_86/dropout/random_uniform/RandomUniformRandomUniform!dropout_86/dropout/Shape:output:0*
T0*(
_output_shapes
:??????????*
dtype021
/dropout_86/dropout/random_uniform/RandomUniform?
!dropout_86/dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *???>2#
!dropout_86/dropout/GreaterEqual/y?
dropout_86/dropout/GreaterEqualGreaterEqual8dropout_86/dropout/random_uniform/RandomUniform:output:0*dropout_86/dropout/GreaterEqual/y:output:0*
T0*(
_output_shapes
:??????????2!
dropout_86/dropout/GreaterEqual?
dropout_86/dropout/CastCast#dropout_86/dropout/GreaterEqual:z:0*

DstT0*

SrcT0
*(
_output_shapes
:??????????2
dropout_86/dropout/Cast?
dropout_86/dropout/Mul_1Muldropout_86/dropout/Mul:z:0dropout_86/dropout/Cast:y:0*
T0*(
_output_shapes
:??????????2
dropout_86/dropout/Mul_1?
dense_86/MatMul/ReadVariableOpReadVariableOp'dense_86_matmul_readvariableop_resource*
_output_shapes
:	?*
dtype02 
dense_86/MatMul/ReadVariableOp?
dense_86/MatMulMatMuldropout_86/dropout/Mul_1:z:0&dense_86/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
dense_86/MatMul?
dense_86/BiasAdd/ReadVariableOpReadVariableOp(dense_86_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02!
dense_86/BiasAdd/ReadVariableOp?
dense_86/BiasAddBiasAdddense_86/MatMul:product:0'dense_86/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
dense_86/BiasAdd?
activation_202/SoftmaxSoftmaxdense_86/BiasAdd:output:0*
T0*'
_output_shapes
:?????????2
activation_202/Softmax?
IdentityIdentity activation_202/Softmax:softmax:0'^batch_normalization_168/AssignNewValue)^batch_normalization_168/AssignNewValue_18^batch_normalization_168/FusedBatchNormV3/ReadVariableOp:^batch_normalization_168/FusedBatchNormV3/ReadVariableOp_1'^batch_normalization_168/ReadVariableOp)^batch_normalization_168/ReadVariableOp_1'^batch_normalization_169/AssignNewValue)^batch_normalization_169/AssignNewValue_18^batch_normalization_169/FusedBatchNormV3/ReadVariableOp:^batch_normalization_169/FusedBatchNormV3/ReadVariableOp_1'^batch_normalization_169/ReadVariableOp)^batch_normalization_169/ReadVariableOp_1'^batch_normalization_170/AssignNewValue)^batch_normalization_170/AssignNewValue_18^batch_normalization_170/FusedBatchNormV3/ReadVariableOp:^batch_normalization_170/FusedBatchNormV3/ReadVariableOp_1'^batch_normalization_170/ReadVariableOp)^batch_normalization_170/ReadVariableOp_1'^batch_normalization_171/AssignNewValue)^batch_normalization_171/AssignNewValue_18^batch_normalization_171/FusedBatchNormV3/ReadVariableOp:^batch_normalization_171/FusedBatchNormV3/ReadVariableOp_1'^batch_normalization_171/ReadVariableOp)^batch_normalization_171/ReadVariableOp_1<^batch_normalization_172/AssignMovingAvg/AssignSubVariableOp7^batch_normalization_172/AssignMovingAvg/ReadVariableOp>^batch_normalization_172/AssignMovingAvg_1/AssignSubVariableOp9^batch_normalization_172/AssignMovingAvg_1/ReadVariableOp1^batch_normalization_172/batchnorm/ReadVariableOp5^batch_normalization_172/batchnorm/mul/ReadVariableOp<^batch_normalization_173/AssignMovingAvg/AssignSubVariableOp7^batch_normalization_173/AssignMovingAvg/ReadVariableOp>^batch_normalization_173/AssignMovingAvg_1/AssignSubVariableOp9^batch_normalization_173/AssignMovingAvg_1/ReadVariableOp1^batch_normalization_173/batchnorm/ReadVariableOp5^batch_normalization_173/batchnorm/mul/ReadVariableOp"^conv2d_112/BiasAdd/ReadVariableOp!^conv2d_112/Conv2D/ReadVariableOp"^conv2d_113/BiasAdd/ReadVariableOp!^conv2d_113/Conv2D/ReadVariableOp"^conv2d_114/BiasAdd/ReadVariableOp!^conv2d_114/Conv2D/ReadVariableOp"^conv2d_115/BiasAdd/ReadVariableOp!^conv2d_115/Conv2D/ReadVariableOp ^dense_84/BiasAdd/ReadVariableOp^dense_84/MatMul/ReadVariableOp ^dense_85/BiasAdd/ReadVariableOp^dense_85/MatMul/ReadVariableOp ^dense_86/BiasAdd/ReadVariableOp^dense_86/MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::2P
&batch_normalization_168/AssignNewValue&batch_normalization_168/AssignNewValue2T
(batch_normalization_168/AssignNewValue_1(batch_normalization_168/AssignNewValue_12r
7batch_normalization_168/FusedBatchNormV3/ReadVariableOp7batch_normalization_168/FusedBatchNormV3/ReadVariableOp2v
9batch_normalization_168/FusedBatchNormV3/ReadVariableOp_19batch_normalization_168/FusedBatchNormV3/ReadVariableOp_12P
&batch_normalization_168/ReadVariableOp&batch_normalization_168/ReadVariableOp2T
(batch_normalization_168/ReadVariableOp_1(batch_normalization_168/ReadVariableOp_12P
&batch_normalization_169/AssignNewValue&batch_normalization_169/AssignNewValue2T
(batch_normalization_169/AssignNewValue_1(batch_normalization_169/AssignNewValue_12r
7batch_normalization_169/FusedBatchNormV3/ReadVariableOp7batch_normalization_169/FusedBatchNormV3/ReadVariableOp2v
9batch_normalization_169/FusedBatchNormV3/ReadVariableOp_19batch_normalization_169/FusedBatchNormV3/ReadVariableOp_12P
&batch_normalization_169/ReadVariableOp&batch_normalization_169/ReadVariableOp2T
(batch_normalization_169/ReadVariableOp_1(batch_normalization_169/ReadVariableOp_12P
&batch_normalization_170/AssignNewValue&batch_normalization_170/AssignNewValue2T
(batch_normalization_170/AssignNewValue_1(batch_normalization_170/AssignNewValue_12r
7batch_normalization_170/FusedBatchNormV3/ReadVariableOp7batch_normalization_170/FusedBatchNormV3/ReadVariableOp2v
9batch_normalization_170/FusedBatchNormV3/ReadVariableOp_19batch_normalization_170/FusedBatchNormV3/ReadVariableOp_12P
&batch_normalization_170/ReadVariableOp&batch_normalization_170/ReadVariableOp2T
(batch_normalization_170/ReadVariableOp_1(batch_normalization_170/ReadVariableOp_12P
&batch_normalization_171/AssignNewValue&batch_normalization_171/AssignNewValue2T
(batch_normalization_171/AssignNewValue_1(batch_normalization_171/AssignNewValue_12r
7batch_normalization_171/FusedBatchNormV3/ReadVariableOp7batch_normalization_171/FusedBatchNormV3/ReadVariableOp2v
9batch_normalization_171/FusedBatchNormV3/ReadVariableOp_19batch_normalization_171/FusedBatchNormV3/ReadVariableOp_12P
&batch_normalization_171/ReadVariableOp&batch_normalization_171/ReadVariableOp2T
(batch_normalization_171/ReadVariableOp_1(batch_normalization_171/ReadVariableOp_12z
;batch_normalization_172/AssignMovingAvg/AssignSubVariableOp;batch_normalization_172/AssignMovingAvg/AssignSubVariableOp2p
6batch_normalization_172/AssignMovingAvg/ReadVariableOp6batch_normalization_172/AssignMovingAvg/ReadVariableOp2~
=batch_normalization_172/AssignMovingAvg_1/AssignSubVariableOp=batch_normalization_172/AssignMovingAvg_1/AssignSubVariableOp2t
8batch_normalization_172/AssignMovingAvg_1/ReadVariableOp8batch_normalization_172/AssignMovingAvg_1/ReadVariableOp2d
0batch_normalization_172/batchnorm/ReadVariableOp0batch_normalization_172/batchnorm/ReadVariableOp2l
4batch_normalization_172/batchnorm/mul/ReadVariableOp4batch_normalization_172/batchnorm/mul/ReadVariableOp2z
;batch_normalization_173/AssignMovingAvg/AssignSubVariableOp;batch_normalization_173/AssignMovingAvg/AssignSubVariableOp2p
6batch_normalization_173/AssignMovingAvg/ReadVariableOp6batch_normalization_173/AssignMovingAvg/ReadVariableOp2~
=batch_normalization_173/AssignMovingAvg_1/AssignSubVariableOp=batch_normalization_173/AssignMovingAvg_1/AssignSubVariableOp2t
8batch_normalization_173/AssignMovingAvg_1/ReadVariableOp8batch_normalization_173/AssignMovingAvg_1/ReadVariableOp2d
0batch_normalization_173/batchnorm/ReadVariableOp0batch_normalization_173/batchnorm/ReadVariableOp2l
4batch_normalization_173/batchnorm/mul/ReadVariableOp4batch_normalization_173/batchnorm/mul/ReadVariableOp2F
!conv2d_112/BiasAdd/ReadVariableOp!conv2d_112/BiasAdd/ReadVariableOp2D
 conv2d_112/Conv2D/ReadVariableOp conv2d_112/Conv2D/ReadVariableOp2F
!conv2d_113/BiasAdd/ReadVariableOp!conv2d_113/BiasAdd/ReadVariableOp2D
 conv2d_113/Conv2D/ReadVariableOp conv2d_113/Conv2D/ReadVariableOp2F
!conv2d_114/BiasAdd/ReadVariableOp!conv2d_114/BiasAdd/ReadVariableOp2D
 conv2d_114/Conv2D/ReadVariableOp conv2d_114/Conv2D/ReadVariableOp2F
!conv2d_115/BiasAdd/ReadVariableOp!conv2d_115/BiasAdd/ReadVariableOp2D
 conv2d_115/Conv2D/ReadVariableOp conv2d_115/Conv2D/ReadVariableOp2B
dense_84/BiasAdd/ReadVariableOpdense_84/BiasAdd/ReadVariableOp2@
dense_84/MatMul/ReadVariableOpdense_84/MatMul/ReadVariableOp2B
dense_85/BiasAdd/ReadVariableOpdense_85/BiasAdd/ReadVariableOp2@
dense_85/MatMul/ReadVariableOpdense_85/MatMul/ReadVariableOp2B
dense_86/BiasAdd/ReadVariableOpdense_86/BiasAdd/ReadVariableOp2@
dense_86/MatMul/ReadVariableOpdense_86/MatMul/ReadVariableOp:W S
/
_output_shapes
:?????????@@
 
_user_specified_nameinputs
?
e
G__inference_dropout_86_layer_call_and_return_conditional_losses_1145010

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
e
G__inference_dropout_86_layer_call_and_return_conditional_losses_1147104

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
?
?
,__inference_conv2d_112_layer_call_fn_1146180

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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_112_layer_call_and_return_conditional_losses_11443172
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
?
?
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_1144365

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
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_1144608

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
?
?
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_1143969

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
?
?
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_1146385

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
?
?
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_1143896

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
?
g
K__inference_activation_200_layer_call_and_return_conditional_losses_1144832

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
?
?
9__inference_batch_normalization_169_layer_call_fn_1146462

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_11444772
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
?
?
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_1144477

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
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_1146274

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
?
?
T__inference_batch_normalization_173_layer_call_and_return_conditional_losses_1147061

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
?0
?
T__inference_batch_normalization_173_layer_call_and_return_conditional_losses_1147041

inputs
assignmovingavg_1147016
assignmovingavg_1_1147022)
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
AssignMovingAvg/decayConst",/job:localhost/replica:0/task:0/device:GPU:0**
_class 
loc:@AssignMovingAvg/1147016*
_output_shapes
: *
dtype0*
valueB
 *
?#<2
AssignMovingAvg/decay?
AssignMovingAvg/ReadVariableOpReadVariableOpassignmovingavg_1147016*
_output_shapes	
:?*
dtype02 
AssignMovingAvg/ReadVariableOp?
AssignMovingAvg/subSub&AssignMovingAvg/ReadVariableOp:value:0moments/Squeeze:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0**
_class 
loc:@AssignMovingAvg/1147016*
_output_shapes	
:?2
AssignMovingAvg/sub?
AssignMovingAvg/mulMulAssignMovingAvg/sub:z:0AssignMovingAvg/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0**
_class 
loc:@AssignMovingAvg/1147016*
_output_shapes	
:?2
AssignMovingAvg/mul?
#AssignMovingAvg/AssignSubVariableOpAssignSubVariableOpassignmovingavg_1147016AssignMovingAvg/mul:z:0^AssignMovingAvg/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0**
_class 
loc:@AssignMovingAvg/1147016*
_output_shapes
 *
dtype02%
#AssignMovingAvg/AssignSubVariableOp?
AssignMovingAvg_1/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*,
_class"
 loc:@AssignMovingAvg_1/1147022*
_output_shapes
: *
dtype0*
valueB
 *
?#<2
AssignMovingAvg_1/decay?
 AssignMovingAvg_1/ReadVariableOpReadVariableOpassignmovingavg_1_1147022*
_output_shapes	
:?*
dtype02"
 AssignMovingAvg_1/ReadVariableOp?
AssignMovingAvg_1/subSub(AssignMovingAvg_1/ReadVariableOp:value:0moments/Squeeze_1:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*,
_class"
 loc:@AssignMovingAvg_1/1147022*
_output_shapes	
:?2
AssignMovingAvg_1/sub?
AssignMovingAvg_1/mulMulAssignMovingAvg_1/sub:z:0 AssignMovingAvg_1/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*,
_class"
 loc:@AssignMovingAvg_1/1147022*
_output_shapes	
:?2
AssignMovingAvg_1/mul?
%AssignMovingAvg_1/AssignSubVariableOpAssignSubVariableOpassignmovingavg_1_1147022AssignMovingAvg_1/mul:z:0!^AssignMovingAvg_1/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*,
_class"
 loc:@AssignMovingAvg_1/1147022*
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
?
?
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_1143780

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
T__inference_batch_normalization_173_layer_call_and_return_conditional_losses_1144259

inputs
assignmovingavg_1144234
assignmovingavg_1_1144240)
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
AssignMovingAvg/decayConst",/job:localhost/replica:0/task:0/device:GPU:0**
_class 
loc:@AssignMovingAvg/1144234*
_output_shapes
: *
dtype0*
valueB
 *
?#<2
AssignMovingAvg/decay?
AssignMovingAvg/ReadVariableOpReadVariableOpassignmovingavg_1144234*
_output_shapes	
:?*
dtype02 
AssignMovingAvg/ReadVariableOp?
AssignMovingAvg/subSub&AssignMovingAvg/ReadVariableOp:value:0moments/Squeeze:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0**
_class 
loc:@AssignMovingAvg/1144234*
_output_shapes	
:?2
AssignMovingAvg/sub?
AssignMovingAvg/mulMulAssignMovingAvg/sub:z:0AssignMovingAvg/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0**
_class 
loc:@AssignMovingAvg/1144234*
_output_shapes	
:?2
AssignMovingAvg/mul?
#AssignMovingAvg/AssignSubVariableOpAssignSubVariableOpassignmovingavg_1144234AssignMovingAvg/mul:z:0^AssignMovingAvg/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0**
_class 
loc:@AssignMovingAvg/1144234*
_output_shapes
 *
dtype02%
#AssignMovingAvg/AssignSubVariableOp?
AssignMovingAvg_1/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*,
_class"
 loc:@AssignMovingAvg_1/1144240*
_output_shapes
: *
dtype0*
valueB
 *
?#<2
AssignMovingAvg_1/decay?
 AssignMovingAvg_1/ReadVariableOpReadVariableOpassignmovingavg_1_1144240*
_output_shapes	
:?*
dtype02"
 AssignMovingAvg_1/ReadVariableOp?
AssignMovingAvg_1/subSub(AssignMovingAvg_1/ReadVariableOp:value:0moments/Squeeze_1:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*,
_class"
 loc:@AssignMovingAvg_1/1144240*
_output_shapes	
:?2
AssignMovingAvg_1/sub?
AssignMovingAvg_1/mulMulAssignMovingAvg_1/sub:z:0 AssignMovingAvg_1/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*,
_class"
 loc:@AssignMovingAvg_1/1144240*
_output_shapes	
:?2
AssignMovingAvg_1/mul?
%AssignMovingAvg_1/AssignSubVariableOpAssignSubVariableOpassignmovingavg_1_1144240AssignMovingAvg_1/mul:z:0!^AssignMovingAvg_1/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*,
_class"
 loc:@AssignMovingAvg_1/1144240*
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
?
L
0__inference_activation_200_layer_call_fn_1146856

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
GPU2*0J 8? *T
fORM
K__inference_activation_200_layer_call_and_return_conditional_losses_11448322
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
?
?
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_1144383

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
e
G__inference_dropout_85_layer_call_and_return_conditional_losses_1146955

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
?
?
9__inference_batch_normalization_169_layer_call_fn_1146475

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_11444952
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
?

*__inference_dense_85_layer_call_fn_1146995

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
GPU2*0J 8? *N
fIRG
E__inference_dense_85_layer_call_and_return_conditional_losses_11449292
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
9__inference_batch_normalization_171_layer_call_fn_1146789

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_11447202
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
??
?
J__inference_sequential_28_layer_call_and_return_conditional_losses_1145468

inputs
conv2d_112_1145364
conv2d_112_1145366#
batch_normalization_168_1145370#
batch_normalization_168_1145372#
batch_normalization_168_1145374#
batch_normalization_168_1145376
conv2d_113_1145379
conv2d_113_1145381#
batch_normalization_169_1145385#
batch_normalization_169_1145387#
batch_normalization_169_1145389#
batch_normalization_169_1145391
conv2d_114_1145395
conv2d_114_1145397#
batch_normalization_170_1145401#
batch_normalization_170_1145403#
batch_normalization_170_1145405#
batch_normalization_170_1145407
conv2d_115_1145410
conv2d_115_1145412#
batch_normalization_171_1145416#
batch_normalization_171_1145418#
batch_normalization_171_1145420#
batch_normalization_171_1145422
dense_84_1145428
dense_84_1145430#
batch_normalization_172_1145434#
batch_normalization_172_1145436#
batch_normalization_172_1145438#
batch_normalization_172_1145440
dense_85_1145445
dense_85_1145447#
batch_normalization_173_1145451#
batch_normalization_173_1145453#
batch_normalization_173_1145455#
batch_normalization_173_1145457
dense_86_1145461
dense_86_1145463
identity??/batch_normalization_168/StatefulPartitionedCall?/batch_normalization_169/StatefulPartitionedCall?/batch_normalization_170/StatefulPartitionedCall?/batch_normalization_171/StatefulPartitionedCall?/batch_normalization_172/StatefulPartitionedCall?/batch_normalization_173/StatefulPartitionedCall?"conv2d_112/StatefulPartitionedCall?"conv2d_113/StatefulPartitionedCall?"conv2d_114/StatefulPartitionedCall?"conv2d_115/StatefulPartitionedCall? dense_84/StatefulPartitionedCall? dense_85/StatefulPartitionedCall? dense_86/StatefulPartitionedCall?
"conv2d_112/StatefulPartitionedCallStatefulPartitionedCallinputsconv2d_112_1145364conv2d_112_1145366*
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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_112_layer_call_and_return_conditional_losses_11443172$
"conv2d_112/StatefulPartitionedCall?
activation_196/PartitionedCallPartitionedCall+conv2d_112/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_196_layer_call_and_return_conditional_losses_11443382 
activation_196/PartitionedCall?
/batch_normalization_168/StatefulPartitionedCallStatefulPartitionedCall'activation_196/PartitionedCall:output:0batch_normalization_168_1145370batch_normalization_168_1145372batch_normalization_168_1145374batch_normalization_168_1145376*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_114438321
/batch_normalization_168/StatefulPartitionedCall?
"conv2d_113/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_168/StatefulPartitionedCall:output:0conv2d_113_1145379conv2d_113_1145381*
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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_113_layer_call_and_return_conditional_losses_11444292$
"conv2d_113/StatefulPartitionedCall?
activation_197/PartitionedCallPartitionedCall+conv2d_113/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_197_layer_call_and_return_conditional_losses_11444502 
activation_197/PartitionedCall?
/batch_normalization_169/StatefulPartitionedCallStatefulPartitionedCall'activation_197/PartitionedCall:output:0batch_normalization_169_1145385batch_normalization_169_1145387batch_normalization_169_1145389batch_normalization_169_1145391*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_114449521
/batch_normalization_169/StatefulPartitionedCall?
 max_pooling2d_56/PartitionedCallPartitionedCall8batch_normalization_169/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *V
fQRO
M__inference_max_pooling2d_56_layer_call_and_return_conditional_losses_11437972"
 max_pooling2d_56/PartitionedCall?
"conv2d_114/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_56/PartitionedCall:output:0conv2d_114_1145395conv2d_114_1145397*
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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_114_layer_call_and_return_conditional_losses_11445422$
"conv2d_114/StatefulPartitionedCall?
activation_198/PartitionedCallPartitionedCall+conv2d_114/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_198_layer_call_and_return_conditional_losses_11445632 
activation_198/PartitionedCall?
/batch_normalization_170/StatefulPartitionedCallStatefulPartitionedCall'activation_198/PartitionedCall:output:0batch_normalization_170_1145401batch_normalization_170_1145403batch_normalization_170_1145405batch_normalization_170_1145407*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_114460821
/batch_normalization_170/StatefulPartitionedCall?
"conv2d_115/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_170/StatefulPartitionedCall:output:0conv2d_115_1145410conv2d_115_1145412*
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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_115_layer_call_and_return_conditional_losses_11446542$
"conv2d_115/StatefulPartitionedCall?
activation_199/PartitionedCallPartitionedCall+conv2d_115/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_199_layer_call_and_return_conditional_losses_11446752 
activation_199/PartitionedCall?
/batch_normalization_171/StatefulPartitionedCallStatefulPartitionedCall'activation_199/PartitionedCall:output:0batch_normalization_171_1145416batch_normalization_171_1145418batch_normalization_171_1145420batch_normalization_171_1145422*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_114472021
/batch_normalization_171/StatefulPartitionedCall?
 max_pooling2d_57/PartitionedCallPartitionedCall8batch_normalization_171/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *V
fQRO
M__inference_max_pooling2d_57_layer_call_and_return_conditional_losses_11440172"
 max_pooling2d_57/PartitionedCall?
dropout_84/PartitionedCallPartitionedCall)max_pooling2d_57/PartitionedCall:output:0*
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
GPU2*0J 8? *P
fKRI
G__inference_dropout_84_layer_call_and_return_conditional_losses_11447742
dropout_84/PartitionedCall?
flatten_56/PartitionedCallPartitionedCall#dropout_84/PartitionedCall:output:0*
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
GPU2*0J 8? *P
fKRI
G__inference_flatten_56_layer_call_and_return_conditional_losses_11447932
flatten_56/PartitionedCall?
 dense_84/StatefulPartitionedCallStatefulPartitionedCall#flatten_56/PartitionedCall:output:0dense_84_1145428dense_84_1145430*
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
GPU2*0J 8? *N
fIRG
E__inference_dense_84_layer_call_and_return_conditional_losses_11448112"
 dense_84/StatefulPartitionedCall?
activation_200/PartitionedCallPartitionedCall)dense_84/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_200_layer_call_and_return_conditional_losses_11448322 
activation_200/PartitionedCall?
/batch_normalization_172/StatefulPartitionedCallStatefulPartitionedCall'activation_200/PartitionedCall:output:0batch_normalization_172_1145434batch_normalization_172_1145436batch_normalization_172_1145438batch_normalization_172_1145440*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_172_layer_call_and_return_conditional_losses_114415221
/batch_normalization_172/StatefulPartitionedCall?
dropout_85/PartitionedCallPartitionedCall8batch_normalization_172/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *P
fKRI
G__inference_dropout_85_layer_call_and_return_conditional_losses_11448922
dropout_85/PartitionedCall?
flatten_57/PartitionedCallPartitionedCall#dropout_85/PartitionedCall:output:0*
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
GPU2*0J 8? *P
fKRI
G__inference_flatten_57_layer_call_and_return_conditional_losses_11449112
flatten_57/PartitionedCall?
 dense_85/StatefulPartitionedCallStatefulPartitionedCall#flatten_57/PartitionedCall:output:0dense_85_1145445dense_85_1145447*
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
GPU2*0J 8? *N
fIRG
E__inference_dense_85_layer_call_and_return_conditional_losses_11449292"
 dense_85/StatefulPartitionedCall?
activation_201/PartitionedCallPartitionedCall)dense_85/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_201_layer_call_and_return_conditional_losses_11449502 
activation_201/PartitionedCall?
/batch_normalization_173/StatefulPartitionedCallStatefulPartitionedCall'activation_201/PartitionedCall:output:0batch_normalization_173_1145451batch_normalization_173_1145453batch_normalization_173_1145455batch_normalization_173_1145457*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_173_layer_call_and_return_conditional_losses_114429221
/batch_normalization_173/StatefulPartitionedCall?
dropout_86/PartitionedCallPartitionedCall8batch_normalization_173/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *P
fKRI
G__inference_dropout_86_layer_call_and_return_conditional_losses_11450102
dropout_86/PartitionedCall?
 dense_86/StatefulPartitionedCallStatefulPartitionedCall#dropout_86/PartitionedCall:output:0dense_86_1145461dense_86_1145463*
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
GPU2*0J 8? *N
fIRG
E__inference_dense_86_layer_call_and_return_conditional_losses_11450332"
 dense_86/StatefulPartitionedCall?
activation_202/PartitionedCallPartitionedCall)dense_86/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_202_layer_call_and_return_conditional_losses_11450542 
activation_202/PartitionedCall?
IdentityIdentity'activation_202/PartitionedCall:output:00^batch_normalization_168/StatefulPartitionedCall0^batch_normalization_169/StatefulPartitionedCall0^batch_normalization_170/StatefulPartitionedCall0^batch_normalization_171/StatefulPartitionedCall0^batch_normalization_172/StatefulPartitionedCall0^batch_normalization_173/StatefulPartitionedCall#^conv2d_112/StatefulPartitionedCall#^conv2d_113/StatefulPartitionedCall#^conv2d_114/StatefulPartitionedCall#^conv2d_115/StatefulPartitionedCall!^dense_84/StatefulPartitionedCall!^dense_85/StatefulPartitionedCall!^dense_86/StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::2b
/batch_normalization_168/StatefulPartitionedCall/batch_normalization_168/StatefulPartitionedCall2b
/batch_normalization_169/StatefulPartitionedCall/batch_normalization_169/StatefulPartitionedCall2b
/batch_normalization_170/StatefulPartitionedCall/batch_normalization_170/StatefulPartitionedCall2b
/batch_normalization_171/StatefulPartitionedCall/batch_normalization_171/StatefulPartitionedCall2b
/batch_normalization_172/StatefulPartitionedCall/batch_normalization_172/StatefulPartitionedCall2b
/batch_normalization_173/StatefulPartitionedCall/batch_normalization_173/StatefulPartitionedCall2H
"conv2d_112/StatefulPartitionedCall"conv2d_112/StatefulPartitionedCall2H
"conv2d_113/StatefulPartitionedCall"conv2d_113/StatefulPartitionedCall2H
"conv2d_114/StatefulPartitionedCall"conv2d_114/StatefulPartitionedCall2H
"conv2d_115/StatefulPartitionedCall"conv2d_115/StatefulPartitionedCall2D
 dense_84/StatefulPartitionedCall dense_84/StatefulPartitionedCall2D
 dense_85/StatefulPartitionedCall dense_85/StatefulPartitionedCall2D
 dense_86/StatefulPartitionedCall dense_86/StatefulPartitionedCall:W S
/
_output_shapes
:?????????@@
 
_user_specified_nameinputs
?
?
T__inference_batch_normalization_172_layer_call_and_return_conditional_losses_1146912

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
?
c
G__inference_flatten_56_layer_call_and_return_conditional_losses_1146822

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
?
g
K__inference_activation_202_layer_call_and_return_conditional_losses_1147138

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
?
?
/__inference_sequential_28_layer_call_fn_1146080

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
GPU2*0J 8? *S
fNRL
J__inference_sequential_28_layer_call_and_return_conditional_losses_11452802
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
?
?
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_1144590

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
?
g
K__inference_activation_196_layer_call_and_return_conditional_losses_1144338

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
?
H
,__inference_dropout_85_layer_call_fn_1146965

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
GPU2*0J 8? *P
fKRI
G__inference_dropout_85_layer_call_and_return_conditional_losses_11448922
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
?
g
K__inference_activation_201_layer_call_and_return_conditional_losses_1147000

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
?
L
0__inference_activation_202_layer_call_fn_1147143

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
GPU2*0J 8? *T
fORM
K__inference_activation_202_layer_call_and_return_conditional_losses_11450542
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
?
?
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_1146542

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
H
,__inference_dropout_86_layer_call_fn_1147114

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
GPU2*0J 8? *P
fKRI
G__inference_dropout_86_layer_call_and_return_conditional_losses_11450102
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
?
?
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_1146228

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
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_1146745

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
?
i
M__inference_max_pooling2d_57_layer_call_and_return_conditional_losses_1144017

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
?
?
9__inference_batch_normalization_169_layer_call_fn_1146398

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_11437492
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
?	
?
G__inference_conv2d_115_layer_call_and_return_conditional_losses_1144654

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
?
f
G__inference_dropout_84_layer_call_and_return_conditional_losses_1144769

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
?
?
9__inference_batch_normalization_170_layer_call_fn_1146555

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_11438652
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
?

*__inference_dense_86_layer_call_fn_1147133

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
GPU2*0J 8? *N
fIRG
E__inference_dense_86_layer_call_and_return_conditional_losses_11450332
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
??
?
J__inference_sequential_28_layer_call_and_return_conditional_losses_1145170
conv2d_112_input
conv2d_112_1145066
conv2d_112_1145068#
batch_normalization_168_1145072#
batch_normalization_168_1145074#
batch_normalization_168_1145076#
batch_normalization_168_1145078
conv2d_113_1145081
conv2d_113_1145083#
batch_normalization_169_1145087#
batch_normalization_169_1145089#
batch_normalization_169_1145091#
batch_normalization_169_1145093
conv2d_114_1145097
conv2d_114_1145099#
batch_normalization_170_1145103#
batch_normalization_170_1145105#
batch_normalization_170_1145107#
batch_normalization_170_1145109
conv2d_115_1145112
conv2d_115_1145114#
batch_normalization_171_1145118#
batch_normalization_171_1145120#
batch_normalization_171_1145122#
batch_normalization_171_1145124
dense_84_1145130
dense_84_1145132#
batch_normalization_172_1145136#
batch_normalization_172_1145138#
batch_normalization_172_1145140#
batch_normalization_172_1145142
dense_85_1145147
dense_85_1145149#
batch_normalization_173_1145153#
batch_normalization_173_1145155#
batch_normalization_173_1145157#
batch_normalization_173_1145159
dense_86_1145163
dense_86_1145165
identity??/batch_normalization_168/StatefulPartitionedCall?/batch_normalization_169/StatefulPartitionedCall?/batch_normalization_170/StatefulPartitionedCall?/batch_normalization_171/StatefulPartitionedCall?/batch_normalization_172/StatefulPartitionedCall?/batch_normalization_173/StatefulPartitionedCall?"conv2d_112/StatefulPartitionedCall?"conv2d_113/StatefulPartitionedCall?"conv2d_114/StatefulPartitionedCall?"conv2d_115/StatefulPartitionedCall? dense_84/StatefulPartitionedCall? dense_85/StatefulPartitionedCall? dense_86/StatefulPartitionedCall?
"conv2d_112/StatefulPartitionedCallStatefulPartitionedCallconv2d_112_inputconv2d_112_1145066conv2d_112_1145068*
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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_112_layer_call_and_return_conditional_losses_11443172$
"conv2d_112/StatefulPartitionedCall?
activation_196/PartitionedCallPartitionedCall+conv2d_112/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_196_layer_call_and_return_conditional_losses_11443382 
activation_196/PartitionedCall?
/batch_normalization_168/StatefulPartitionedCallStatefulPartitionedCall'activation_196/PartitionedCall:output:0batch_normalization_168_1145072batch_normalization_168_1145074batch_normalization_168_1145076batch_normalization_168_1145078*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_114438321
/batch_normalization_168/StatefulPartitionedCall?
"conv2d_113/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_168/StatefulPartitionedCall:output:0conv2d_113_1145081conv2d_113_1145083*
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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_113_layer_call_and_return_conditional_losses_11444292$
"conv2d_113/StatefulPartitionedCall?
activation_197/PartitionedCallPartitionedCall+conv2d_113/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_197_layer_call_and_return_conditional_losses_11444502 
activation_197/PartitionedCall?
/batch_normalization_169/StatefulPartitionedCallStatefulPartitionedCall'activation_197/PartitionedCall:output:0batch_normalization_169_1145087batch_normalization_169_1145089batch_normalization_169_1145091batch_normalization_169_1145093*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_114449521
/batch_normalization_169/StatefulPartitionedCall?
 max_pooling2d_56/PartitionedCallPartitionedCall8batch_normalization_169/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *V
fQRO
M__inference_max_pooling2d_56_layer_call_and_return_conditional_losses_11437972"
 max_pooling2d_56/PartitionedCall?
"conv2d_114/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_56/PartitionedCall:output:0conv2d_114_1145097conv2d_114_1145099*
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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_114_layer_call_and_return_conditional_losses_11445422$
"conv2d_114/StatefulPartitionedCall?
activation_198/PartitionedCallPartitionedCall+conv2d_114/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_198_layer_call_and_return_conditional_losses_11445632 
activation_198/PartitionedCall?
/batch_normalization_170/StatefulPartitionedCallStatefulPartitionedCall'activation_198/PartitionedCall:output:0batch_normalization_170_1145103batch_normalization_170_1145105batch_normalization_170_1145107batch_normalization_170_1145109*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_114460821
/batch_normalization_170/StatefulPartitionedCall?
"conv2d_115/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_170/StatefulPartitionedCall:output:0conv2d_115_1145112conv2d_115_1145114*
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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_115_layer_call_and_return_conditional_losses_11446542$
"conv2d_115/StatefulPartitionedCall?
activation_199/PartitionedCallPartitionedCall+conv2d_115/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_199_layer_call_and_return_conditional_losses_11446752 
activation_199/PartitionedCall?
/batch_normalization_171/StatefulPartitionedCallStatefulPartitionedCall'activation_199/PartitionedCall:output:0batch_normalization_171_1145118batch_normalization_171_1145120batch_normalization_171_1145122batch_normalization_171_1145124*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_114472021
/batch_normalization_171/StatefulPartitionedCall?
 max_pooling2d_57/PartitionedCallPartitionedCall8batch_normalization_171/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *V
fQRO
M__inference_max_pooling2d_57_layer_call_and_return_conditional_losses_11440172"
 max_pooling2d_57/PartitionedCall?
dropout_84/PartitionedCallPartitionedCall)max_pooling2d_57/PartitionedCall:output:0*
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
GPU2*0J 8? *P
fKRI
G__inference_dropout_84_layer_call_and_return_conditional_losses_11447742
dropout_84/PartitionedCall?
flatten_56/PartitionedCallPartitionedCall#dropout_84/PartitionedCall:output:0*
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
GPU2*0J 8? *P
fKRI
G__inference_flatten_56_layer_call_and_return_conditional_losses_11447932
flatten_56/PartitionedCall?
 dense_84/StatefulPartitionedCallStatefulPartitionedCall#flatten_56/PartitionedCall:output:0dense_84_1145130dense_84_1145132*
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
GPU2*0J 8? *N
fIRG
E__inference_dense_84_layer_call_and_return_conditional_losses_11448112"
 dense_84/StatefulPartitionedCall?
activation_200/PartitionedCallPartitionedCall)dense_84/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_200_layer_call_and_return_conditional_losses_11448322 
activation_200/PartitionedCall?
/batch_normalization_172/StatefulPartitionedCallStatefulPartitionedCall'activation_200/PartitionedCall:output:0batch_normalization_172_1145136batch_normalization_172_1145138batch_normalization_172_1145140batch_normalization_172_1145142*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_172_layer_call_and_return_conditional_losses_114415221
/batch_normalization_172/StatefulPartitionedCall?
dropout_85/PartitionedCallPartitionedCall8batch_normalization_172/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *P
fKRI
G__inference_dropout_85_layer_call_and_return_conditional_losses_11448922
dropout_85/PartitionedCall?
flatten_57/PartitionedCallPartitionedCall#dropout_85/PartitionedCall:output:0*
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
GPU2*0J 8? *P
fKRI
G__inference_flatten_57_layer_call_and_return_conditional_losses_11449112
flatten_57/PartitionedCall?
 dense_85/StatefulPartitionedCallStatefulPartitionedCall#flatten_57/PartitionedCall:output:0dense_85_1145147dense_85_1145149*
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
GPU2*0J 8? *N
fIRG
E__inference_dense_85_layer_call_and_return_conditional_losses_11449292"
 dense_85/StatefulPartitionedCall?
activation_201/PartitionedCallPartitionedCall)dense_85/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_201_layer_call_and_return_conditional_losses_11449502 
activation_201/PartitionedCall?
/batch_normalization_173/StatefulPartitionedCallStatefulPartitionedCall'activation_201/PartitionedCall:output:0batch_normalization_173_1145153batch_normalization_173_1145155batch_normalization_173_1145157batch_normalization_173_1145159*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_173_layer_call_and_return_conditional_losses_114429221
/batch_normalization_173/StatefulPartitionedCall?
dropout_86/PartitionedCallPartitionedCall8batch_normalization_173/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *P
fKRI
G__inference_dropout_86_layer_call_and_return_conditional_losses_11450102
dropout_86/PartitionedCall?
 dense_86/StatefulPartitionedCallStatefulPartitionedCall#dropout_86/PartitionedCall:output:0dense_86_1145163dense_86_1145165*
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
GPU2*0J 8? *N
fIRG
E__inference_dense_86_layer_call_and_return_conditional_losses_11450332"
 dense_86/StatefulPartitionedCall?
activation_202/PartitionedCallPartitionedCall)dense_86/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_202_layer_call_and_return_conditional_losses_11450542 
activation_202/PartitionedCall?
IdentityIdentity'activation_202/PartitionedCall:output:00^batch_normalization_168/StatefulPartitionedCall0^batch_normalization_169/StatefulPartitionedCall0^batch_normalization_170/StatefulPartitionedCall0^batch_normalization_171/StatefulPartitionedCall0^batch_normalization_172/StatefulPartitionedCall0^batch_normalization_173/StatefulPartitionedCall#^conv2d_112/StatefulPartitionedCall#^conv2d_113/StatefulPartitionedCall#^conv2d_114/StatefulPartitionedCall#^conv2d_115/StatefulPartitionedCall!^dense_84/StatefulPartitionedCall!^dense_85/StatefulPartitionedCall!^dense_86/StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::2b
/batch_normalization_168/StatefulPartitionedCall/batch_normalization_168/StatefulPartitionedCall2b
/batch_normalization_169/StatefulPartitionedCall/batch_normalization_169/StatefulPartitionedCall2b
/batch_normalization_170/StatefulPartitionedCall/batch_normalization_170/StatefulPartitionedCall2b
/batch_normalization_171/StatefulPartitionedCall/batch_normalization_171/StatefulPartitionedCall2b
/batch_normalization_172/StatefulPartitionedCall/batch_normalization_172/StatefulPartitionedCall2b
/batch_normalization_173/StatefulPartitionedCall/batch_normalization_173/StatefulPartitionedCall2H
"conv2d_112/StatefulPartitionedCall"conv2d_112/StatefulPartitionedCall2H
"conv2d_113/StatefulPartitionedCall"conv2d_113/StatefulPartitionedCall2H
"conv2d_114/StatefulPartitionedCall"conv2d_114/StatefulPartitionedCall2H
"conv2d_115/StatefulPartitionedCall"conv2d_115/StatefulPartitionedCall2D
 dense_84/StatefulPartitionedCall dense_84/StatefulPartitionedCall2D
 dense_85/StatefulPartitionedCall dense_85/StatefulPartitionedCall2D
 dense_86/StatefulPartitionedCall dense_86/StatefulPartitionedCall:a ]
/
_output_shapes
:?????????@@
*
_user_specified_nameconv2d_112_input
?
?
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_1146367

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
?
?
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_1146292

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
?
?
9__inference_batch_normalization_170_layer_call_fn_1146632

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_11446082
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
?
H
,__inference_flatten_57_layer_call_fn_1146976

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
GPU2*0J 8? *P
fKRI
G__inference_flatten_57_layer_call_and_return_conditional_losses_11449112
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
?
L
0__inference_activation_199_layer_call_fn_1146661

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
GPU2*0J 8? *T
fORM
K__inference_activation_199_layer_call_and_return_conditional_losses_11446752
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
??
?
J__inference_sequential_28_layer_call_and_return_conditional_losses_1145063
conv2d_112_input
conv2d_112_1144328
conv2d_112_1144330#
batch_normalization_168_1144410#
batch_normalization_168_1144412#
batch_normalization_168_1144414#
batch_normalization_168_1144416
conv2d_113_1144440
conv2d_113_1144442#
batch_normalization_169_1144522#
batch_normalization_169_1144524#
batch_normalization_169_1144526#
batch_normalization_169_1144528
conv2d_114_1144553
conv2d_114_1144555#
batch_normalization_170_1144635#
batch_normalization_170_1144637#
batch_normalization_170_1144639#
batch_normalization_170_1144641
conv2d_115_1144665
conv2d_115_1144667#
batch_normalization_171_1144747#
batch_normalization_171_1144749#
batch_normalization_171_1144751#
batch_normalization_171_1144753
dense_84_1144822
dense_84_1144824#
batch_normalization_172_1144866#
batch_normalization_172_1144868#
batch_normalization_172_1144870#
batch_normalization_172_1144872
dense_85_1144940
dense_85_1144942#
batch_normalization_173_1144984#
batch_normalization_173_1144986#
batch_normalization_173_1144988#
batch_normalization_173_1144990
dense_86_1145044
dense_86_1145046
identity??/batch_normalization_168/StatefulPartitionedCall?/batch_normalization_169/StatefulPartitionedCall?/batch_normalization_170/StatefulPartitionedCall?/batch_normalization_171/StatefulPartitionedCall?/batch_normalization_172/StatefulPartitionedCall?/batch_normalization_173/StatefulPartitionedCall?"conv2d_112/StatefulPartitionedCall?"conv2d_113/StatefulPartitionedCall?"conv2d_114/StatefulPartitionedCall?"conv2d_115/StatefulPartitionedCall? dense_84/StatefulPartitionedCall? dense_85/StatefulPartitionedCall? dense_86/StatefulPartitionedCall?"dropout_84/StatefulPartitionedCall?"dropout_85/StatefulPartitionedCall?"dropout_86/StatefulPartitionedCall?
"conv2d_112/StatefulPartitionedCallStatefulPartitionedCallconv2d_112_inputconv2d_112_1144328conv2d_112_1144330*
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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_112_layer_call_and_return_conditional_losses_11443172$
"conv2d_112/StatefulPartitionedCall?
activation_196/PartitionedCallPartitionedCall+conv2d_112/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_196_layer_call_and_return_conditional_losses_11443382 
activation_196/PartitionedCall?
/batch_normalization_168/StatefulPartitionedCallStatefulPartitionedCall'activation_196/PartitionedCall:output:0batch_normalization_168_1144410batch_normalization_168_1144412batch_normalization_168_1144414batch_normalization_168_1144416*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_114436521
/batch_normalization_168/StatefulPartitionedCall?
"conv2d_113/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_168/StatefulPartitionedCall:output:0conv2d_113_1144440conv2d_113_1144442*
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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_113_layer_call_and_return_conditional_losses_11444292$
"conv2d_113/StatefulPartitionedCall?
activation_197/PartitionedCallPartitionedCall+conv2d_113/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_197_layer_call_and_return_conditional_losses_11444502 
activation_197/PartitionedCall?
/batch_normalization_169/StatefulPartitionedCallStatefulPartitionedCall'activation_197/PartitionedCall:output:0batch_normalization_169_1144522batch_normalization_169_1144524batch_normalization_169_1144526batch_normalization_169_1144528*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_114447721
/batch_normalization_169/StatefulPartitionedCall?
 max_pooling2d_56/PartitionedCallPartitionedCall8batch_normalization_169/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *V
fQRO
M__inference_max_pooling2d_56_layer_call_and_return_conditional_losses_11437972"
 max_pooling2d_56/PartitionedCall?
"conv2d_114/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_56/PartitionedCall:output:0conv2d_114_1144553conv2d_114_1144555*
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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_114_layer_call_and_return_conditional_losses_11445422$
"conv2d_114/StatefulPartitionedCall?
activation_198/PartitionedCallPartitionedCall+conv2d_114/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_198_layer_call_and_return_conditional_losses_11445632 
activation_198/PartitionedCall?
/batch_normalization_170/StatefulPartitionedCallStatefulPartitionedCall'activation_198/PartitionedCall:output:0batch_normalization_170_1144635batch_normalization_170_1144637batch_normalization_170_1144639batch_normalization_170_1144641*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_114459021
/batch_normalization_170/StatefulPartitionedCall?
"conv2d_115/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_170/StatefulPartitionedCall:output:0conv2d_115_1144665conv2d_115_1144667*
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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_115_layer_call_and_return_conditional_losses_11446542$
"conv2d_115/StatefulPartitionedCall?
activation_199/PartitionedCallPartitionedCall+conv2d_115/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_199_layer_call_and_return_conditional_losses_11446752 
activation_199/PartitionedCall?
/batch_normalization_171/StatefulPartitionedCallStatefulPartitionedCall'activation_199/PartitionedCall:output:0batch_normalization_171_1144747batch_normalization_171_1144749batch_normalization_171_1144751batch_normalization_171_1144753*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_114470221
/batch_normalization_171/StatefulPartitionedCall?
 max_pooling2d_57/PartitionedCallPartitionedCall8batch_normalization_171/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *V
fQRO
M__inference_max_pooling2d_57_layer_call_and_return_conditional_losses_11440172"
 max_pooling2d_57/PartitionedCall?
"dropout_84/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_57/PartitionedCall:output:0*
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
GPU2*0J 8? *P
fKRI
G__inference_dropout_84_layer_call_and_return_conditional_losses_11447692$
"dropout_84/StatefulPartitionedCall?
flatten_56/PartitionedCallPartitionedCall+dropout_84/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *P
fKRI
G__inference_flatten_56_layer_call_and_return_conditional_losses_11447932
flatten_56/PartitionedCall?
 dense_84/StatefulPartitionedCallStatefulPartitionedCall#flatten_56/PartitionedCall:output:0dense_84_1144822dense_84_1144824*
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
GPU2*0J 8? *N
fIRG
E__inference_dense_84_layer_call_and_return_conditional_losses_11448112"
 dense_84/StatefulPartitionedCall?
activation_200/PartitionedCallPartitionedCall)dense_84/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_200_layer_call_and_return_conditional_losses_11448322 
activation_200/PartitionedCall?
/batch_normalization_172/StatefulPartitionedCallStatefulPartitionedCall'activation_200/PartitionedCall:output:0batch_normalization_172_1144866batch_normalization_172_1144868batch_normalization_172_1144870batch_normalization_172_1144872*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_172_layer_call_and_return_conditional_losses_114411921
/batch_normalization_172/StatefulPartitionedCall?
"dropout_85/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_172/StatefulPartitionedCall:output:0#^dropout_84/StatefulPartitionedCall*
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
GPU2*0J 8? *P
fKRI
G__inference_dropout_85_layer_call_and_return_conditional_losses_11448872$
"dropout_85/StatefulPartitionedCall?
flatten_57/PartitionedCallPartitionedCall+dropout_85/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *P
fKRI
G__inference_flatten_57_layer_call_and_return_conditional_losses_11449112
flatten_57/PartitionedCall?
 dense_85/StatefulPartitionedCallStatefulPartitionedCall#flatten_57/PartitionedCall:output:0dense_85_1144940dense_85_1144942*
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
GPU2*0J 8? *N
fIRG
E__inference_dense_85_layer_call_and_return_conditional_losses_11449292"
 dense_85/StatefulPartitionedCall?
activation_201/PartitionedCallPartitionedCall)dense_85/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_201_layer_call_and_return_conditional_losses_11449502 
activation_201/PartitionedCall?
/batch_normalization_173/StatefulPartitionedCallStatefulPartitionedCall'activation_201/PartitionedCall:output:0batch_normalization_173_1144984batch_normalization_173_1144986batch_normalization_173_1144988batch_normalization_173_1144990*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_173_layer_call_and_return_conditional_losses_114425921
/batch_normalization_173/StatefulPartitionedCall?
"dropout_86/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_173/StatefulPartitionedCall:output:0#^dropout_85/StatefulPartitionedCall*
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
GPU2*0J 8? *P
fKRI
G__inference_dropout_86_layer_call_and_return_conditional_losses_11450052$
"dropout_86/StatefulPartitionedCall?
 dense_86/StatefulPartitionedCallStatefulPartitionedCall+dropout_86/StatefulPartitionedCall:output:0dense_86_1145044dense_86_1145046*
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
GPU2*0J 8? *N
fIRG
E__inference_dense_86_layer_call_and_return_conditional_losses_11450332"
 dense_86/StatefulPartitionedCall?
activation_202/PartitionedCallPartitionedCall)dense_86/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_202_layer_call_and_return_conditional_losses_11450542 
activation_202/PartitionedCall?
IdentityIdentity'activation_202/PartitionedCall:output:00^batch_normalization_168/StatefulPartitionedCall0^batch_normalization_169/StatefulPartitionedCall0^batch_normalization_170/StatefulPartitionedCall0^batch_normalization_171/StatefulPartitionedCall0^batch_normalization_172/StatefulPartitionedCall0^batch_normalization_173/StatefulPartitionedCall#^conv2d_112/StatefulPartitionedCall#^conv2d_113/StatefulPartitionedCall#^conv2d_114/StatefulPartitionedCall#^conv2d_115/StatefulPartitionedCall!^dense_84/StatefulPartitionedCall!^dense_85/StatefulPartitionedCall!^dense_86/StatefulPartitionedCall#^dropout_84/StatefulPartitionedCall#^dropout_85/StatefulPartitionedCall#^dropout_86/StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::2b
/batch_normalization_168/StatefulPartitionedCall/batch_normalization_168/StatefulPartitionedCall2b
/batch_normalization_169/StatefulPartitionedCall/batch_normalization_169/StatefulPartitionedCall2b
/batch_normalization_170/StatefulPartitionedCall/batch_normalization_170/StatefulPartitionedCall2b
/batch_normalization_171/StatefulPartitionedCall/batch_normalization_171/StatefulPartitionedCall2b
/batch_normalization_172/StatefulPartitionedCall/batch_normalization_172/StatefulPartitionedCall2b
/batch_normalization_173/StatefulPartitionedCall/batch_normalization_173/StatefulPartitionedCall2H
"conv2d_112/StatefulPartitionedCall"conv2d_112/StatefulPartitionedCall2H
"conv2d_113/StatefulPartitionedCall"conv2d_113/StatefulPartitionedCall2H
"conv2d_114/StatefulPartitionedCall"conv2d_114/StatefulPartitionedCall2H
"conv2d_115/StatefulPartitionedCall"conv2d_115/StatefulPartitionedCall2D
 dense_84/StatefulPartitionedCall dense_84/StatefulPartitionedCall2D
 dense_85/StatefulPartitionedCall dense_85/StatefulPartitionedCall2D
 dense_86/StatefulPartitionedCall dense_86/StatefulPartitionedCall2H
"dropout_84/StatefulPartitionedCall"dropout_84/StatefulPartitionedCall2H
"dropout_85/StatefulPartitionedCall"dropout_85/StatefulPartitionedCall2H
"dropout_86/StatefulPartitionedCall"dropout_86/StatefulPartitionedCall:a ]
/
_output_shapes
:?????????@@
*
_user_specified_nameconv2d_112_input
?	
?
G__inference_conv2d_113_layer_call_and_return_conditional_losses_1144429

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
?
g
K__inference_activation_197_layer_call_and_return_conditional_losses_1144450

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
?
?
9__inference_batch_normalization_173_layer_call_fn_1147074

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_173_layer_call_and_return_conditional_losses_11442592
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
9__inference_batch_normalization_172_layer_call_fn_1146925

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_172_layer_call_and_return_conditional_losses_11441192
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
?
g
K__inference_activation_199_layer_call_and_return_conditional_losses_1146656

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
?
?
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_1143676

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
?
?
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_1146606

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
?	
?
E__inference_dense_86_layer_call_and_return_conditional_losses_1147124

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
?
?
/__inference_sequential_28_layer_call_fn_1146161

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
GPU2*0J 8? *S
fNRL
J__inference_sequential_28_layer_call_and_return_conditional_losses_11454682
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
?
g
K__inference_activation_202_layer_call_and_return_conditional_losses_1145054

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
?	
?
E__inference_dense_85_layer_call_and_return_conditional_losses_1146986

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
?
?
9__inference_batch_normalization_170_layer_call_fn_1146619

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_11445902
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
9__inference_batch_normalization_168_layer_call_fn_1146305

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_11443652
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
?
?
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_1146524

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
?
g
K__inference_activation_198_layer_call_and_return_conditional_losses_1144563

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
?	
?
E__inference_dense_85_layer_call_and_return_conditional_losses_1144929

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
G__inference_conv2d_112_layer_call_and_return_conditional_losses_1146171

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
?
i
M__inference_max_pooling2d_56_layer_call_and_return_conditional_losses_1143797

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
?	
?
E__inference_dense_86_layer_call_and_return_conditional_losses_1145033

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
?
?
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_1144000

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
?
?
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_1143749

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
?
9__inference_batch_normalization_172_layer_call_fn_1146938

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_172_layer_call_and_return_conditional_losses_11441522
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
?
?
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_1146449

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
?
?
%__inference_signature_wrapper_1145638
conv2d_112_input
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
StatefulPartitionedCallStatefulPartitionedCallconv2d_112_inputunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
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
GPU2*0J 8? *+
f&R$
"__inference__wrapped_model_11435832
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::22
StatefulPartitionedCallStatefulPartitionedCall:a ]
/
_output_shapes
:?????????@@
*
_user_specified_nameconv2d_112_input
?
?
9__inference_batch_normalization_168_layer_call_fn_1146254

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_11436762
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
?
f
G__inference_dropout_85_layer_call_and_return_conditional_losses_1146950

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
9__inference_batch_normalization_170_layer_call_fn_1146568

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_11438962
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
?	
?
E__inference_dense_84_layer_call_and_return_conditional_losses_1146837

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
?
L
0__inference_activation_197_layer_call_fn_1146347

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
GPU2*0J 8? *T
fORM
K__inference_activation_197_layer_call_and_return_conditional_losses_11444502
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
??
?.
 __inference__traced_save_1147463
file_prefix0
,savev2_conv2d_112_kernel_read_readvariableop.
*savev2_conv2d_112_bias_read_readvariableop<
8savev2_batch_normalization_168_gamma_read_readvariableop;
7savev2_batch_normalization_168_beta_read_readvariableopB
>savev2_batch_normalization_168_moving_mean_read_readvariableopF
Bsavev2_batch_normalization_168_moving_variance_read_readvariableop0
,savev2_conv2d_113_kernel_read_readvariableop.
*savev2_conv2d_113_bias_read_readvariableop<
8savev2_batch_normalization_169_gamma_read_readvariableop;
7savev2_batch_normalization_169_beta_read_readvariableopB
>savev2_batch_normalization_169_moving_mean_read_readvariableopF
Bsavev2_batch_normalization_169_moving_variance_read_readvariableop0
,savev2_conv2d_114_kernel_read_readvariableop.
*savev2_conv2d_114_bias_read_readvariableop<
8savev2_batch_normalization_170_gamma_read_readvariableop;
7savev2_batch_normalization_170_beta_read_readvariableopB
>savev2_batch_normalization_170_moving_mean_read_readvariableopF
Bsavev2_batch_normalization_170_moving_variance_read_readvariableop0
,savev2_conv2d_115_kernel_read_readvariableop.
*savev2_conv2d_115_bias_read_readvariableop<
8savev2_batch_normalization_171_gamma_read_readvariableop;
7savev2_batch_normalization_171_beta_read_readvariableopB
>savev2_batch_normalization_171_moving_mean_read_readvariableopF
Bsavev2_batch_normalization_171_moving_variance_read_readvariableop.
*savev2_dense_84_kernel_read_readvariableop,
(savev2_dense_84_bias_read_readvariableop<
8savev2_batch_normalization_172_gamma_read_readvariableop;
7savev2_batch_normalization_172_beta_read_readvariableopB
>savev2_batch_normalization_172_moving_mean_read_readvariableopF
Bsavev2_batch_normalization_172_moving_variance_read_readvariableop.
*savev2_dense_85_kernel_read_readvariableop,
(savev2_dense_85_bias_read_readvariableop<
8savev2_batch_normalization_173_gamma_read_readvariableop;
7savev2_batch_normalization_173_beta_read_readvariableopB
>savev2_batch_normalization_173_moving_mean_read_readvariableopF
Bsavev2_batch_normalization_173_moving_variance_read_readvariableop.
*savev2_dense_86_kernel_read_readvariableop,
(savev2_dense_86_bias_read_readvariableop(
$savev2_adam_iter_read_readvariableop	*
&savev2_adam_beta_1_read_readvariableop*
&savev2_adam_beta_2_read_readvariableop)
%savev2_adam_decay_read_readvariableop1
-savev2_adam_learning_rate_read_readvariableop$
 savev2_total_read_readvariableop$
 savev2_count_read_readvariableop&
"savev2_total_1_read_readvariableop&
"savev2_count_1_read_readvariableop7
3savev2_adam_conv2d_112_kernel_m_read_readvariableop5
1savev2_adam_conv2d_112_bias_m_read_readvariableopC
?savev2_adam_batch_normalization_168_gamma_m_read_readvariableopB
>savev2_adam_batch_normalization_168_beta_m_read_readvariableop7
3savev2_adam_conv2d_113_kernel_m_read_readvariableop5
1savev2_adam_conv2d_113_bias_m_read_readvariableopC
?savev2_adam_batch_normalization_169_gamma_m_read_readvariableopB
>savev2_adam_batch_normalization_169_beta_m_read_readvariableop7
3savev2_adam_conv2d_114_kernel_m_read_readvariableop5
1savev2_adam_conv2d_114_bias_m_read_readvariableopC
?savev2_adam_batch_normalization_170_gamma_m_read_readvariableopB
>savev2_adam_batch_normalization_170_beta_m_read_readvariableop7
3savev2_adam_conv2d_115_kernel_m_read_readvariableop5
1savev2_adam_conv2d_115_bias_m_read_readvariableopC
?savev2_adam_batch_normalization_171_gamma_m_read_readvariableopB
>savev2_adam_batch_normalization_171_beta_m_read_readvariableop5
1savev2_adam_dense_84_kernel_m_read_readvariableop3
/savev2_adam_dense_84_bias_m_read_readvariableopC
?savev2_adam_batch_normalization_172_gamma_m_read_readvariableopB
>savev2_adam_batch_normalization_172_beta_m_read_readvariableop5
1savev2_adam_dense_85_kernel_m_read_readvariableop3
/savev2_adam_dense_85_bias_m_read_readvariableopC
?savev2_adam_batch_normalization_173_gamma_m_read_readvariableopB
>savev2_adam_batch_normalization_173_beta_m_read_readvariableop5
1savev2_adam_dense_86_kernel_m_read_readvariableop3
/savev2_adam_dense_86_bias_m_read_readvariableop7
3savev2_adam_conv2d_112_kernel_v_read_readvariableop5
1savev2_adam_conv2d_112_bias_v_read_readvariableopC
?savev2_adam_batch_normalization_168_gamma_v_read_readvariableopB
>savev2_adam_batch_normalization_168_beta_v_read_readvariableop7
3savev2_adam_conv2d_113_kernel_v_read_readvariableop5
1savev2_adam_conv2d_113_bias_v_read_readvariableopC
?savev2_adam_batch_normalization_169_gamma_v_read_readvariableopB
>savev2_adam_batch_normalization_169_beta_v_read_readvariableop7
3savev2_adam_conv2d_114_kernel_v_read_readvariableop5
1savev2_adam_conv2d_114_bias_v_read_readvariableopC
?savev2_adam_batch_normalization_170_gamma_v_read_readvariableopB
>savev2_adam_batch_normalization_170_beta_v_read_readvariableop7
3savev2_adam_conv2d_115_kernel_v_read_readvariableop5
1savev2_adam_conv2d_115_bias_v_read_readvariableopC
?savev2_adam_batch_normalization_171_gamma_v_read_readvariableopB
>savev2_adam_batch_normalization_171_beta_v_read_readvariableop5
1savev2_adam_dense_84_kernel_v_read_readvariableop3
/savev2_adam_dense_84_bias_v_read_readvariableopC
?savev2_adam_batch_normalization_172_gamma_v_read_readvariableopB
>savev2_adam_batch_normalization_172_beta_v_read_readvariableop5
1savev2_adam_dense_85_kernel_v_read_readvariableop3
/savev2_adam_dense_85_bias_v_read_readvariableopC
?savev2_adam_batch_normalization_173_gamma_v_read_readvariableopB
>savev2_adam_batch_normalization_173_beta_v_read_readvariableop5
1savev2_adam_dense_86_kernel_v_read_readvariableop3
/savev2_adam_dense_86_bias_v_read_readvariableop
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
SaveV2SaveV2ShardedFilename:filename:0SaveV2/tensor_names:output:0 SaveV2/shape_and_slices:output:0,savev2_conv2d_112_kernel_read_readvariableop*savev2_conv2d_112_bias_read_readvariableop8savev2_batch_normalization_168_gamma_read_readvariableop7savev2_batch_normalization_168_beta_read_readvariableop>savev2_batch_normalization_168_moving_mean_read_readvariableopBsavev2_batch_normalization_168_moving_variance_read_readvariableop,savev2_conv2d_113_kernel_read_readvariableop*savev2_conv2d_113_bias_read_readvariableop8savev2_batch_normalization_169_gamma_read_readvariableop7savev2_batch_normalization_169_beta_read_readvariableop>savev2_batch_normalization_169_moving_mean_read_readvariableopBsavev2_batch_normalization_169_moving_variance_read_readvariableop,savev2_conv2d_114_kernel_read_readvariableop*savev2_conv2d_114_bias_read_readvariableop8savev2_batch_normalization_170_gamma_read_readvariableop7savev2_batch_normalization_170_beta_read_readvariableop>savev2_batch_normalization_170_moving_mean_read_readvariableopBsavev2_batch_normalization_170_moving_variance_read_readvariableop,savev2_conv2d_115_kernel_read_readvariableop*savev2_conv2d_115_bias_read_readvariableop8savev2_batch_normalization_171_gamma_read_readvariableop7savev2_batch_normalization_171_beta_read_readvariableop>savev2_batch_normalization_171_moving_mean_read_readvariableopBsavev2_batch_normalization_171_moving_variance_read_readvariableop*savev2_dense_84_kernel_read_readvariableop(savev2_dense_84_bias_read_readvariableop8savev2_batch_normalization_172_gamma_read_readvariableop7savev2_batch_normalization_172_beta_read_readvariableop>savev2_batch_normalization_172_moving_mean_read_readvariableopBsavev2_batch_normalization_172_moving_variance_read_readvariableop*savev2_dense_85_kernel_read_readvariableop(savev2_dense_85_bias_read_readvariableop8savev2_batch_normalization_173_gamma_read_readvariableop7savev2_batch_normalization_173_beta_read_readvariableop>savev2_batch_normalization_173_moving_mean_read_readvariableopBsavev2_batch_normalization_173_moving_variance_read_readvariableop*savev2_dense_86_kernel_read_readvariableop(savev2_dense_86_bias_read_readvariableop$savev2_adam_iter_read_readvariableop&savev2_adam_beta_1_read_readvariableop&savev2_adam_beta_2_read_readvariableop%savev2_adam_decay_read_readvariableop-savev2_adam_learning_rate_read_readvariableop savev2_total_read_readvariableop savev2_count_read_readvariableop"savev2_total_1_read_readvariableop"savev2_count_1_read_readvariableop3savev2_adam_conv2d_112_kernel_m_read_readvariableop1savev2_adam_conv2d_112_bias_m_read_readvariableop?savev2_adam_batch_normalization_168_gamma_m_read_readvariableop>savev2_adam_batch_normalization_168_beta_m_read_readvariableop3savev2_adam_conv2d_113_kernel_m_read_readvariableop1savev2_adam_conv2d_113_bias_m_read_readvariableop?savev2_adam_batch_normalization_169_gamma_m_read_readvariableop>savev2_adam_batch_normalization_169_beta_m_read_readvariableop3savev2_adam_conv2d_114_kernel_m_read_readvariableop1savev2_adam_conv2d_114_bias_m_read_readvariableop?savev2_adam_batch_normalization_170_gamma_m_read_readvariableop>savev2_adam_batch_normalization_170_beta_m_read_readvariableop3savev2_adam_conv2d_115_kernel_m_read_readvariableop1savev2_adam_conv2d_115_bias_m_read_readvariableop?savev2_adam_batch_normalization_171_gamma_m_read_readvariableop>savev2_adam_batch_normalization_171_beta_m_read_readvariableop1savev2_adam_dense_84_kernel_m_read_readvariableop/savev2_adam_dense_84_bias_m_read_readvariableop?savev2_adam_batch_normalization_172_gamma_m_read_readvariableop>savev2_adam_batch_normalization_172_beta_m_read_readvariableop1savev2_adam_dense_85_kernel_m_read_readvariableop/savev2_adam_dense_85_bias_m_read_readvariableop?savev2_adam_batch_normalization_173_gamma_m_read_readvariableop>savev2_adam_batch_normalization_173_beta_m_read_readvariableop1savev2_adam_dense_86_kernel_m_read_readvariableop/savev2_adam_dense_86_bias_m_read_readvariableop3savev2_adam_conv2d_112_kernel_v_read_readvariableop1savev2_adam_conv2d_112_bias_v_read_readvariableop?savev2_adam_batch_normalization_168_gamma_v_read_readvariableop>savev2_adam_batch_normalization_168_beta_v_read_readvariableop3savev2_adam_conv2d_113_kernel_v_read_readvariableop1savev2_adam_conv2d_113_bias_v_read_readvariableop?savev2_adam_batch_normalization_169_gamma_v_read_readvariableop>savev2_adam_batch_normalization_169_beta_v_read_readvariableop3savev2_adam_conv2d_114_kernel_v_read_readvariableop1savev2_adam_conv2d_114_bias_v_read_readvariableop?savev2_adam_batch_normalization_170_gamma_v_read_readvariableop>savev2_adam_batch_normalization_170_beta_v_read_readvariableop3savev2_adam_conv2d_115_kernel_v_read_readvariableop1savev2_adam_conv2d_115_bias_v_read_readvariableop?savev2_adam_batch_normalization_171_gamma_v_read_readvariableop>savev2_adam_batch_normalization_171_beta_v_read_readvariableop1savev2_adam_dense_84_kernel_v_read_readvariableop/savev2_adam_dense_84_bias_v_read_readvariableop?savev2_adam_batch_normalization_172_gamma_v_read_readvariableop>savev2_adam_batch_normalization_172_beta_v_read_readvariableop1savev2_adam_dense_85_kernel_v_read_readvariableop/savev2_adam_dense_85_bias_v_read_readvariableop?savev2_adam_batch_normalization_173_gamma_v_read_readvariableop>savev2_adam_batch_normalization_173_beta_v_read_readvariableop1savev2_adam_dense_86_kernel_v_read_readvariableop/savev2_adam_dense_86_bias_v_read_readvariableopsavev2_const"/device:CPU:0*
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
?
?
9__inference_batch_normalization_173_layer_call_fn_1147087

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_173_layer_call_and_return_conditional_losses_11442922
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
?
f
G__inference_dropout_84_layer_call_and_return_conditional_losses_1146801

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
?
N
2__inference_max_pooling2d_57_layer_call_fn_1144023

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
GPU2*0J 8? *V
fQRO
M__inference_max_pooling2d_57_layer_call_and_return_conditional_losses_11440172
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
?
e
,__inference_dropout_84_layer_call_fn_1146811

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
GPU2*0J 8? *P
fKRI
G__inference_dropout_84_layer_call_and_return_conditional_losses_11447692
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
?
?
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_1144702

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
?
?
9__inference_batch_normalization_169_layer_call_fn_1146411

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_11437802
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
g
K__inference_activation_196_layer_call_and_return_conditional_losses_1146185

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
?
L
0__inference_activation_201_layer_call_fn_1147005

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
GPU2*0J 8? *T
fORM
K__inference_activation_201_layer_call_and_return_conditional_losses_11449502
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
?
g
K__inference_activation_200_layer_call_and_return_conditional_losses_1146851

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
?
e
,__inference_dropout_85_layer_call_fn_1146960

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
GPU2*0J 8? *P
fKRI
G__inference_dropout_85_layer_call_and_return_conditional_losses_11448872
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
?
?
/__inference_sequential_28_layer_call_fn_1145359
conv2d_112_input
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
StatefulPartitionedCallStatefulPartitionedCallconv2d_112_inputunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
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
GPU2*0J 8? *S
fNRL
J__inference_sequential_28_layer_call_and_return_conditional_losses_11452802
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::22
StatefulPartitionedCallStatefulPartitionedCall:a ]
/
_output_shapes
:?????????@@
*
_user_specified_nameconv2d_112_input
?
?
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_1146431

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
?
g
K__inference_activation_197_layer_call_and_return_conditional_losses_1146342

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
?
?
,__inference_conv2d_114_layer_call_fn_1146494

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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_114_layer_call_and_return_conditional_losses_11445422
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
?
?
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_1146588

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
?0
?
T__inference_batch_normalization_172_layer_call_and_return_conditional_losses_1144119

inputs
assignmovingavg_1144094
assignmovingavg_1_1144100)
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
AssignMovingAvg/decayConst",/job:localhost/replica:0/task:0/device:GPU:0**
_class 
loc:@AssignMovingAvg/1144094*
_output_shapes
: *
dtype0*
valueB
 *
?#<2
AssignMovingAvg/decay?
AssignMovingAvg/ReadVariableOpReadVariableOpassignmovingavg_1144094*
_output_shapes	
:?*
dtype02 
AssignMovingAvg/ReadVariableOp?
AssignMovingAvg/subSub&AssignMovingAvg/ReadVariableOp:value:0moments/Squeeze:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0**
_class 
loc:@AssignMovingAvg/1144094*
_output_shapes	
:?2
AssignMovingAvg/sub?
AssignMovingAvg/mulMulAssignMovingAvg/sub:z:0AssignMovingAvg/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0**
_class 
loc:@AssignMovingAvg/1144094*
_output_shapes	
:?2
AssignMovingAvg/mul?
#AssignMovingAvg/AssignSubVariableOpAssignSubVariableOpassignmovingavg_1144094AssignMovingAvg/mul:z:0^AssignMovingAvg/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0**
_class 
loc:@AssignMovingAvg/1144094*
_output_shapes
 *
dtype02%
#AssignMovingAvg/AssignSubVariableOp?
AssignMovingAvg_1/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*,
_class"
 loc:@AssignMovingAvg_1/1144100*
_output_shapes
: *
dtype0*
valueB
 *
?#<2
AssignMovingAvg_1/decay?
 AssignMovingAvg_1/ReadVariableOpReadVariableOpassignmovingavg_1_1144100*
_output_shapes	
:?*
dtype02"
 AssignMovingAvg_1/ReadVariableOp?
AssignMovingAvg_1/subSub(AssignMovingAvg_1/ReadVariableOp:value:0moments/Squeeze_1:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*,
_class"
 loc:@AssignMovingAvg_1/1144100*
_output_shapes	
:?2
AssignMovingAvg_1/sub?
AssignMovingAvg_1/mulMulAssignMovingAvg_1/sub:z:0 AssignMovingAvg_1/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*,
_class"
 loc:@AssignMovingAvg_1/1144100*
_output_shapes	
:?2
AssignMovingAvg_1/mul?
%AssignMovingAvg_1/AssignSubVariableOpAssignSubVariableOpassignmovingavg_1_1144100AssignMovingAvg_1/mul:z:0!^AssignMovingAvg_1/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*,
_class"
 loc:@AssignMovingAvg_1/1144100*
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
?
L
0__inference_activation_196_layer_call_fn_1146190

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
GPU2*0J 8? *T
fORM
K__inference_activation_196_layer_call_and_return_conditional_losses_11443382
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
?
?
9__inference_batch_normalization_168_layer_call_fn_1146318

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_11443832
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
9__inference_batch_normalization_171_layer_call_fn_1146776

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_11447022
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
?
g
K__inference_activation_198_layer_call_and_return_conditional_losses_1146499

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
?
?
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_1143865

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
?
e
G__inference_dropout_84_layer_call_and_return_conditional_losses_1146806

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
?
f
G__inference_dropout_86_layer_call_and_return_conditional_losses_1145005

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
9__inference_batch_normalization_168_layer_call_fn_1146241

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_11436452
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
?
?
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_1146763

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
?
?
T__inference_batch_normalization_172_layer_call_and_return_conditional_losses_1144152

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
??
?
J__inference_sequential_28_layer_call_and_return_conditional_losses_1145280

inputs
conv2d_112_1145176
conv2d_112_1145178#
batch_normalization_168_1145182#
batch_normalization_168_1145184#
batch_normalization_168_1145186#
batch_normalization_168_1145188
conv2d_113_1145191
conv2d_113_1145193#
batch_normalization_169_1145197#
batch_normalization_169_1145199#
batch_normalization_169_1145201#
batch_normalization_169_1145203
conv2d_114_1145207
conv2d_114_1145209#
batch_normalization_170_1145213#
batch_normalization_170_1145215#
batch_normalization_170_1145217#
batch_normalization_170_1145219
conv2d_115_1145222
conv2d_115_1145224#
batch_normalization_171_1145228#
batch_normalization_171_1145230#
batch_normalization_171_1145232#
batch_normalization_171_1145234
dense_84_1145240
dense_84_1145242#
batch_normalization_172_1145246#
batch_normalization_172_1145248#
batch_normalization_172_1145250#
batch_normalization_172_1145252
dense_85_1145257
dense_85_1145259#
batch_normalization_173_1145263#
batch_normalization_173_1145265#
batch_normalization_173_1145267#
batch_normalization_173_1145269
dense_86_1145273
dense_86_1145275
identity??/batch_normalization_168/StatefulPartitionedCall?/batch_normalization_169/StatefulPartitionedCall?/batch_normalization_170/StatefulPartitionedCall?/batch_normalization_171/StatefulPartitionedCall?/batch_normalization_172/StatefulPartitionedCall?/batch_normalization_173/StatefulPartitionedCall?"conv2d_112/StatefulPartitionedCall?"conv2d_113/StatefulPartitionedCall?"conv2d_114/StatefulPartitionedCall?"conv2d_115/StatefulPartitionedCall? dense_84/StatefulPartitionedCall? dense_85/StatefulPartitionedCall? dense_86/StatefulPartitionedCall?"dropout_84/StatefulPartitionedCall?"dropout_85/StatefulPartitionedCall?"dropout_86/StatefulPartitionedCall?
"conv2d_112/StatefulPartitionedCallStatefulPartitionedCallinputsconv2d_112_1145176conv2d_112_1145178*
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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_112_layer_call_and_return_conditional_losses_11443172$
"conv2d_112/StatefulPartitionedCall?
activation_196/PartitionedCallPartitionedCall+conv2d_112/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_196_layer_call_and_return_conditional_losses_11443382 
activation_196/PartitionedCall?
/batch_normalization_168/StatefulPartitionedCallStatefulPartitionedCall'activation_196/PartitionedCall:output:0batch_normalization_168_1145182batch_normalization_168_1145184batch_normalization_168_1145186batch_normalization_168_1145188*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_114436521
/batch_normalization_168/StatefulPartitionedCall?
"conv2d_113/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_168/StatefulPartitionedCall:output:0conv2d_113_1145191conv2d_113_1145193*
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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_113_layer_call_and_return_conditional_losses_11444292$
"conv2d_113/StatefulPartitionedCall?
activation_197/PartitionedCallPartitionedCall+conv2d_113/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_197_layer_call_and_return_conditional_losses_11444502 
activation_197/PartitionedCall?
/batch_normalization_169/StatefulPartitionedCallStatefulPartitionedCall'activation_197/PartitionedCall:output:0batch_normalization_169_1145197batch_normalization_169_1145199batch_normalization_169_1145201batch_normalization_169_1145203*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_114447721
/batch_normalization_169/StatefulPartitionedCall?
 max_pooling2d_56/PartitionedCallPartitionedCall8batch_normalization_169/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *V
fQRO
M__inference_max_pooling2d_56_layer_call_and_return_conditional_losses_11437972"
 max_pooling2d_56/PartitionedCall?
"conv2d_114/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_56/PartitionedCall:output:0conv2d_114_1145207conv2d_114_1145209*
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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_114_layer_call_and_return_conditional_losses_11445422$
"conv2d_114/StatefulPartitionedCall?
activation_198/PartitionedCallPartitionedCall+conv2d_114/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_198_layer_call_and_return_conditional_losses_11445632 
activation_198/PartitionedCall?
/batch_normalization_170/StatefulPartitionedCallStatefulPartitionedCall'activation_198/PartitionedCall:output:0batch_normalization_170_1145213batch_normalization_170_1145215batch_normalization_170_1145217batch_normalization_170_1145219*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_114459021
/batch_normalization_170/StatefulPartitionedCall?
"conv2d_115/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_170/StatefulPartitionedCall:output:0conv2d_115_1145222conv2d_115_1145224*
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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_115_layer_call_and_return_conditional_losses_11446542$
"conv2d_115/StatefulPartitionedCall?
activation_199/PartitionedCallPartitionedCall+conv2d_115/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_199_layer_call_and_return_conditional_losses_11446752 
activation_199/PartitionedCall?
/batch_normalization_171/StatefulPartitionedCallStatefulPartitionedCall'activation_199/PartitionedCall:output:0batch_normalization_171_1145228batch_normalization_171_1145230batch_normalization_171_1145232batch_normalization_171_1145234*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_114470221
/batch_normalization_171/StatefulPartitionedCall?
 max_pooling2d_57/PartitionedCallPartitionedCall8batch_normalization_171/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *V
fQRO
M__inference_max_pooling2d_57_layer_call_and_return_conditional_losses_11440172"
 max_pooling2d_57/PartitionedCall?
"dropout_84/StatefulPartitionedCallStatefulPartitionedCall)max_pooling2d_57/PartitionedCall:output:0*
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
GPU2*0J 8? *P
fKRI
G__inference_dropout_84_layer_call_and_return_conditional_losses_11447692$
"dropout_84/StatefulPartitionedCall?
flatten_56/PartitionedCallPartitionedCall+dropout_84/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *P
fKRI
G__inference_flatten_56_layer_call_and_return_conditional_losses_11447932
flatten_56/PartitionedCall?
 dense_84/StatefulPartitionedCallStatefulPartitionedCall#flatten_56/PartitionedCall:output:0dense_84_1145240dense_84_1145242*
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
GPU2*0J 8? *N
fIRG
E__inference_dense_84_layer_call_and_return_conditional_losses_11448112"
 dense_84/StatefulPartitionedCall?
activation_200/PartitionedCallPartitionedCall)dense_84/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_200_layer_call_and_return_conditional_losses_11448322 
activation_200/PartitionedCall?
/batch_normalization_172/StatefulPartitionedCallStatefulPartitionedCall'activation_200/PartitionedCall:output:0batch_normalization_172_1145246batch_normalization_172_1145248batch_normalization_172_1145250batch_normalization_172_1145252*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_172_layer_call_and_return_conditional_losses_114411921
/batch_normalization_172/StatefulPartitionedCall?
"dropout_85/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_172/StatefulPartitionedCall:output:0#^dropout_84/StatefulPartitionedCall*
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
GPU2*0J 8? *P
fKRI
G__inference_dropout_85_layer_call_and_return_conditional_losses_11448872$
"dropout_85/StatefulPartitionedCall?
flatten_57/PartitionedCallPartitionedCall+dropout_85/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *P
fKRI
G__inference_flatten_57_layer_call_and_return_conditional_losses_11449112
flatten_57/PartitionedCall?
 dense_85/StatefulPartitionedCallStatefulPartitionedCall#flatten_57/PartitionedCall:output:0dense_85_1145257dense_85_1145259*
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
GPU2*0J 8? *N
fIRG
E__inference_dense_85_layer_call_and_return_conditional_losses_11449292"
 dense_85/StatefulPartitionedCall?
activation_201/PartitionedCallPartitionedCall)dense_85/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_201_layer_call_and_return_conditional_losses_11449502 
activation_201/PartitionedCall?
/batch_normalization_173/StatefulPartitionedCallStatefulPartitionedCall'activation_201/PartitionedCall:output:0batch_normalization_173_1145263batch_normalization_173_1145265batch_normalization_173_1145267batch_normalization_173_1145269*
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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_173_layer_call_and_return_conditional_losses_114425921
/batch_normalization_173/StatefulPartitionedCall?
"dropout_86/StatefulPartitionedCallStatefulPartitionedCall8batch_normalization_173/StatefulPartitionedCall:output:0#^dropout_85/StatefulPartitionedCall*
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
GPU2*0J 8? *P
fKRI
G__inference_dropout_86_layer_call_and_return_conditional_losses_11450052$
"dropout_86/StatefulPartitionedCall?
 dense_86/StatefulPartitionedCallStatefulPartitionedCall+dropout_86/StatefulPartitionedCall:output:0dense_86_1145273dense_86_1145275*
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
GPU2*0J 8? *N
fIRG
E__inference_dense_86_layer_call_and_return_conditional_losses_11450332"
 dense_86/StatefulPartitionedCall?
activation_202/PartitionedCallPartitionedCall)dense_86/StatefulPartitionedCall:output:0*
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
GPU2*0J 8? *T
fORM
K__inference_activation_202_layer_call_and_return_conditional_losses_11450542 
activation_202/PartitionedCall?
IdentityIdentity'activation_202/PartitionedCall:output:00^batch_normalization_168/StatefulPartitionedCall0^batch_normalization_169/StatefulPartitionedCall0^batch_normalization_170/StatefulPartitionedCall0^batch_normalization_171/StatefulPartitionedCall0^batch_normalization_172/StatefulPartitionedCall0^batch_normalization_173/StatefulPartitionedCall#^conv2d_112/StatefulPartitionedCall#^conv2d_113/StatefulPartitionedCall#^conv2d_114/StatefulPartitionedCall#^conv2d_115/StatefulPartitionedCall!^dense_84/StatefulPartitionedCall!^dense_85/StatefulPartitionedCall!^dense_86/StatefulPartitionedCall#^dropout_84/StatefulPartitionedCall#^dropout_85/StatefulPartitionedCall#^dropout_86/StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::2b
/batch_normalization_168/StatefulPartitionedCall/batch_normalization_168/StatefulPartitionedCall2b
/batch_normalization_169/StatefulPartitionedCall/batch_normalization_169/StatefulPartitionedCall2b
/batch_normalization_170/StatefulPartitionedCall/batch_normalization_170/StatefulPartitionedCall2b
/batch_normalization_171/StatefulPartitionedCall/batch_normalization_171/StatefulPartitionedCall2b
/batch_normalization_172/StatefulPartitionedCall/batch_normalization_172/StatefulPartitionedCall2b
/batch_normalization_173/StatefulPartitionedCall/batch_normalization_173/StatefulPartitionedCall2H
"conv2d_112/StatefulPartitionedCall"conv2d_112/StatefulPartitionedCall2H
"conv2d_113/StatefulPartitionedCall"conv2d_113/StatefulPartitionedCall2H
"conv2d_114/StatefulPartitionedCall"conv2d_114/StatefulPartitionedCall2H
"conv2d_115/StatefulPartitionedCall"conv2d_115/StatefulPartitionedCall2D
 dense_84/StatefulPartitionedCall dense_84/StatefulPartitionedCall2D
 dense_85/StatefulPartitionedCall dense_85/StatefulPartitionedCall2D
 dense_86/StatefulPartitionedCall dense_86/StatefulPartitionedCall2H
"dropout_84/StatefulPartitionedCall"dropout_84/StatefulPartitionedCall2H
"dropout_85/StatefulPartitionedCall"dropout_85/StatefulPartitionedCall2H
"dropout_86/StatefulPartitionedCall"dropout_86/StatefulPartitionedCall:W S
/
_output_shapes
:?????????@@
 
_user_specified_nameinputs
?	
?
G__inference_conv2d_114_layer_call_and_return_conditional_losses_1146485

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
?0
?
T__inference_batch_normalization_172_layer_call_and_return_conditional_losses_1146892

inputs
assignmovingavg_1146867
assignmovingavg_1_1146873)
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
AssignMovingAvg/decayConst",/job:localhost/replica:0/task:0/device:GPU:0**
_class 
loc:@AssignMovingAvg/1146867*
_output_shapes
: *
dtype0*
valueB
 *
?#<2
AssignMovingAvg/decay?
AssignMovingAvg/ReadVariableOpReadVariableOpassignmovingavg_1146867*
_output_shapes	
:?*
dtype02 
AssignMovingAvg/ReadVariableOp?
AssignMovingAvg/subSub&AssignMovingAvg/ReadVariableOp:value:0moments/Squeeze:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0**
_class 
loc:@AssignMovingAvg/1146867*
_output_shapes	
:?2
AssignMovingAvg/sub?
AssignMovingAvg/mulMulAssignMovingAvg/sub:z:0AssignMovingAvg/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0**
_class 
loc:@AssignMovingAvg/1146867*
_output_shapes	
:?2
AssignMovingAvg/mul?
#AssignMovingAvg/AssignSubVariableOpAssignSubVariableOpassignmovingavg_1146867AssignMovingAvg/mul:z:0^AssignMovingAvg/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0**
_class 
loc:@AssignMovingAvg/1146867*
_output_shapes
 *
dtype02%
#AssignMovingAvg/AssignSubVariableOp?
AssignMovingAvg_1/decayConst",/job:localhost/replica:0/task:0/device:GPU:0*,
_class"
 loc:@AssignMovingAvg_1/1146873*
_output_shapes
: *
dtype0*
valueB
 *
?#<2
AssignMovingAvg_1/decay?
 AssignMovingAvg_1/ReadVariableOpReadVariableOpassignmovingavg_1_1146873*
_output_shapes	
:?*
dtype02"
 AssignMovingAvg_1/ReadVariableOp?
AssignMovingAvg_1/subSub(AssignMovingAvg_1/ReadVariableOp:value:0moments/Squeeze_1:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*,
_class"
 loc:@AssignMovingAvg_1/1146873*
_output_shapes	
:?2
AssignMovingAvg_1/sub?
AssignMovingAvg_1/mulMulAssignMovingAvg_1/sub:z:0 AssignMovingAvg_1/decay:output:0",/job:localhost/replica:0/task:0/device:GPU:0*
T0*,
_class"
 loc:@AssignMovingAvg_1/1146873*
_output_shapes	
:?2
AssignMovingAvg_1/mul?
%AssignMovingAvg_1/AssignSubVariableOpAssignSubVariableOpassignmovingavg_1_1146873AssignMovingAvg_1/mul:z:0!^AssignMovingAvg_1/ReadVariableOp",/job:localhost/replica:0/task:0/device:GPU:0*,
_class"
 loc:@AssignMovingAvg_1/1146873*
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
?
?
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_1144720

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
?
g
K__inference_activation_199_layer_call_and_return_conditional_losses_1144675

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
?
?
,__inference_conv2d_115_layer_call_fn_1146651

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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_115_layer_call_and_return_conditional_losses_11446542
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
L
0__inference_activation_198_layer_call_fn_1146504

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
GPU2*0J 8? *T
fORM
K__inference_activation_198_layer_call_and_return_conditional_losses_11445632
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
?	
?
G__inference_conv2d_114_layer_call_and_return_conditional_losses_1144542

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
?

*__inference_dense_84_layer_call_fn_1146846

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
GPU2*0J 8? *N
fIRG
E__inference_dense_84_layer_call_and_return_conditional_losses_11448112
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
E__inference_dense_84_layer_call_and_return_conditional_losses_1144811

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
?
?
9__inference_batch_normalization_171_layer_call_fn_1146725

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
GPU2*0J 8? *]
fXRV
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_11440002
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
?
?
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_1146210

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
??
?9
#__inference__traced_restore_1147770
file_prefix&
"assignvariableop_conv2d_112_kernel&
"assignvariableop_1_conv2d_112_bias4
0assignvariableop_2_batch_normalization_168_gamma3
/assignvariableop_3_batch_normalization_168_beta:
6assignvariableop_4_batch_normalization_168_moving_mean>
:assignvariableop_5_batch_normalization_168_moving_variance(
$assignvariableop_6_conv2d_113_kernel&
"assignvariableop_7_conv2d_113_bias4
0assignvariableop_8_batch_normalization_169_gamma3
/assignvariableop_9_batch_normalization_169_beta;
7assignvariableop_10_batch_normalization_169_moving_mean?
;assignvariableop_11_batch_normalization_169_moving_variance)
%assignvariableop_12_conv2d_114_kernel'
#assignvariableop_13_conv2d_114_bias5
1assignvariableop_14_batch_normalization_170_gamma4
0assignvariableop_15_batch_normalization_170_beta;
7assignvariableop_16_batch_normalization_170_moving_mean?
;assignvariableop_17_batch_normalization_170_moving_variance)
%assignvariableop_18_conv2d_115_kernel'
#assignvariableop_19_conv2d_115_bias5
1assignvariableop_20_batch_normalization_171_gamma4
0assignvariableop_21_batch_normalization_171_beta;
7assignvariableop_22_batch_normalization_171_moving_mean?
;assignvariableop_23_batch_normalization_171_moving_variance'
#assignvariableop_24_dense_84_kernel%
!assignvariableop_25_dense_84_bias5
1assignvariableop_26_batch_normalization_172_gamma4
0assignvariableop_27_batch_normalization_172_beta;
7assignvariableop_28_batch_normalization_172_moving_mean?
;assignvariableop_29_batch_normalization_172_moving_variance'
#assignvariableop_30_dense_85_kernel%
!assignvariableop_31_dense_85_bias5
1assignvariableop_32_batch_normalization_173_gamma4
0assignvariableop_33_batch_normalization_173_beta;
7assignvariableop_34_batch_normalization_173_moving_mean?
;assignvariableop_35_batch_normalization_173_moving_variance'
#assignvariableop_36_dense_86_kernel%
!assignvariableop_37_dense_86_bias!
assignvariableop_38_adam_iter#
assignvariableop_39_adam_beta_1#
assignvariableop_40_adam_beta_2"
assignvariableop_41_adam_decay*
&assignvariableop_42_adam_learning_rate
assignvariableop_43_total
assignvariableop_44_count
assignvariableop_45_total_1
assignvariableop_46_count_10
,assignvariableop_47_adam_conv2d_112_kernel_m.
*assignvariableop_48_adam_conv2d_112_bias_m<
8assignvariableop_49_adam_batch_normalization_168_gamma_m;
7assignvariableop_50_adam_batch_normalization_168_beta_m0
,assignvariableop_51_adam_conv2d_113_kernel_m.
*assignvariableop_52_adam_conv2d_113_bias_m<
8assignvariableop_53_adam_batch_normalization_169_gamma_m;
7assignvariableop_54_adam_batch_normalization_169_beta_m0
,assignvariableop_55_adam_conv2d_114_kernel_m.
*assignvariableop_56_adam_conv2d_114_bias_m<
8assignvariableop_57_adam_batch_normalization_170_gamma_m;
7assignvariableop_58_adam_batch_normalization_170_beta_m0
,assignvariableop_59_adam_conv2d_115_kernel_m.
*assignvariableop_60_adam_conv2d_115_bias_m<
8assignvariableop_61_adam_batch_normalization_171_gamma_m;
7assignvariableop_62_adam_batch_normalization_171_beta_m.
*assignvariableop_63_adam_dense_84_kernel_m,
(assignvariableop_64_adam_dense_84_bias_m<
8assignvariableop_65_adam_batch_normalization_172_gamma_m;
7assignvariableop_66_adam_batch_normalization_172_beta_m.
*assignvariableop_67_adam_dense_85_kernel_m,
(assignvariableop_68_adam_dense_85_bias_m<
8assignvariableop_69_adam_batch_normalization_173_gamma_m;
7assignvariableop_70_adam_batch_normalization_173_beta_m.
*assignvariableop_71_adam_dense_86_kernel_m,
(assignvariableop_72_adam_dense_86_bias_m0
,assignvariableop_73_adam_conv2d_112_kernel_v.
*assignvariableop_74_adam_conv2d_112_bias_v<
8assignvariableop_75_adam_batch_normalization_168_gamma_v;
7assignvariableop_76_adam_batch_normalization_168_beta_v0
,assignvariableop_77_adam_conv2d_113_kernel_v.
*assignvariableop_78_adam_conv2d_113_bias_v<
8assignvariableop_79_adam_batch_normalization_169_gamma_v;
7assignvariableop_80_adam_batch_normalization_169_beta_v0
,assignvariableop_81_adam_conv2d_114_kernel_v.
*assignvariableop_82_adam_conv2d_114_bias_v<
8assignvariableop_83_adam_batch_normalization_170_gamma_v;
7assignvariableop_84_adam_batch_normalization_170_beta_v0
,assignvariableop_85_adam_conv2d_115_kernel_v.
*assignvariableop_86_adam_conv2d_115_bias_v<
8assignvariableop_87_adam_batch_normalization_171_gamma_v;
7assignvariableop_88_adam_batch_normalization_171_beta_v.
*assignvariableop_89_adam_dense_84_kernel_v,
(assignvariableop_90_adam_dense_84_bias_v<
8assignvariableop_91_adam_batch_normalization_172_gamma_v;
7assignvariableop_92_adam_batch_normalization_172_beta_v.
*assignvariableop_93_adam_dense_85_kernel_v,
(assignvariableop_94_adam_dense_85_bias_v<
8assignvariableop_95_adam_batch_normalization_173_gamma_v;
7assignvariableop_96_adam_batch_normalization_173_beta_v.
*assignvariableop_97_adam_dense_86_kernel_v,
(assignvariableop_98_adam_dense_86_bias_v
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
AssignVariableOpAssignVariableOp"assignvariableop_conv2d_112_kernelIdentity:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOpk

Identity_1IdentityRestoreV2:tensors:1"/device:CPU:0*
T0*
_output_shapes
:2

Identity_1?
AssignVariableOp_1AssignVariableOp"assignvariableop_1_conv2d_112_biasIdentity_1:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_1k

Identity_2IdentityRestoreV2:tensors:2"/device:CPU:0*
T0*
_output_shapes
:2

Identity_2?
AssignVariableOp_2AssignVariableOp0assignvariableop_2_batch_normalization_168_gammaIdentity_2:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_2k

Identity_3IdentityRestoreV2:tensors:3"/device:CPU:0*
T0*
_output_shapes
:2

Identity_3?
AssignVariableOp_3AssignVariableOp/assignvariableop_3_batch_normalization_168_betaIdentity_3:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_3k

Identity_4IdentityRestoreV2:tensors:4"/device:CPU:0*
T0*
_output_shapes
:2

Identity_4?
AssignVariableOp_4AssignVariableOp6assignvariableop_4_batch_normalization_168_moving_meanIdentity_4:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_4k

Identity_5IdentityRestoreV2:tensors:5"/device:CPU:0*
T0*
_output_shapes
:2

Identity_5?
AssignVariableOp_5AssignVariableOp:assignvariableop_5_batch_normalization_168_moving_varianceIdentity_5:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_5k

Identity_6IdentityRestoreV2:tensors:6"/device:CPU:0*
T0*
_output_shapes
:2

Identity_6?
AssignVariableOp_6AssignVariableOp$assignvariableop_6_conv2d_113_kernelIdentity_6:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_6k

Identity_7IdentityRestoreV2:tensors:7"/device:CPU:0*
T0*
_output_shapes
:2

Identity_7?
AssignVariableOp_7AssignVariableOp"assignvariableop_7_conv2d_113_biasIdentity_7:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_7k

Identity_8IdentityRestoreV2:tensors:8"/device:CPU:0*
T0*
_output_shapes
:2

Identity_8?
AssignVariableOp_8AssignVariableOp0assignvariableop_8_batch_normalization_169_gammaIdentity_8:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_8k

Identity_9IdentityRestoreV2:tensors:9"/device:CPU:0*
T0*
_output_shapes
:2

Identity_9?
AssignVariableOp_9AssignVariableOp/assignvariableop_9_batch_normalization_169_betaIdentity_9:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_9n
Identity_10IdentityRestoreV2:tensors:10"/device:CPU:0*
T0*
_output_shapes
:2
Identity_10?
AssignVariableOp_10AssignVariableOp7assignvariableop_10_batch_normalization_169_moving_meanIdentity_10:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_10n
Identity_11IdentityRestoreV2:tensors:11"/device:CPU:0*
T0*
_output_shapes
:2
Identity_11?
AssignVariableOp_11AssignVariableOp;assignvariableop_11_batch_normalization_169_moving_varianceIdentity_11:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_11n
Identity_12IdentityRestoreV2:tensors:12"/device:CPU:0*
T0*
_output_shapes
:2
Identity_12?
AssignVariableOp_12AssignVariableOp%assignvariableop_12_conv2d_114_kernelIdentity_12:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_12n
Identity_13IdentityRestoreV2:tensors:13"/device:CPU:0*
T0*
_output_shapes
:2
Identity_13?
AssignVariableOp_13AssignVariableOp#assignvariableop_13_conv2d_114_biasIdentity_13:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_13n
Identity_14IdentityRestoreV2:tensors:14"/device:CPU:0*
T0*
_output_shapes
:2
Identity_14?
AssignVariableOp_14AssignVariableOp1assignvariableop_14_batch_normalization_170_gammaIdentity_14:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_14n
Identity_15IdentityRestoreV2:tensors:15"/device:CPU:0*
T0*
_output_shapes
:2
Identity_15?
AssignVariableOp_15AssignVariableOp0assignvariableop_15_batch_normalization_170_betaIdentity_15:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_15n
Identity_16IdentityRestoreV2:tensors:16"/device:CPU:0*
T0*
_output_shapes
:2
Identity_16?
AssignVariableOp_16AssignVariableOp7assignvariableop_16_batch_normalization_170_moving_meanIdentity_16:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_16n
Identity_17IdentityRestoreV2:tensors:17"/device:CPU:0*
T0*
_output_shapes
:2
Identity_17?
AssignVariableOp_17AssignVariableOp;assignvariableop_17_batch_normalization_170_moving_varianceIdentity_17:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_17n
Identity_18IdentityRestoreV2:tensors:18"/device:CPU:0*
T0*
_output_shapes
:2
Identity_18?
AssignVariableOp_18AssignVariableOp%assignvariableop_18_conv2d_115_kernelIdentity_18:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_18n
Identity_19IdentityRestoreV2:tensors:19"/device:CPU:0*
T0*
_output_shapes
:2
Identity_19?
AssignVariableOp_19AssignVariableOp#assignvariableop_19_conv2d_115_biasIdentity_19:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_19n
Identity_20IdentityRestoreV2:tensors:20"/device:CPU:0*
T0*
_output_shapes
:2
Identity_20?
AssignVariableOp_20AssignVariableOp1assignvariableop_20_batch_normalization_171_gammaIdentity_20:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_20n
Identity_21IdentityRestoreV2:tensors:21"/device:CPU:0*
T0*
_output_shapes
:2
Identity_21?
AssignVariableOp_21AssignVariableOp0assignvariableop_21_batch_normalization_171_betaIdentity_21:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_21n
Identity_22IdentityRestoreV2:tensors:22"/device:CPU:0*
T0*
_output_shapes
:2
Identity_22?
AssignVariableOp_22AssignVariableOp7assignvariableop_22_batch_normalization_171_moving_meanIdentity_22:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_22n
Identity_23IdentityRestoreV2:tensors:23"/device:CPU:0*
T0*
_output_shapes
:2
Identity_23?
AssignVariableOp_23AssignVariableOp;assignvariableop_23_batch_normalization_171_moving_varianceIdentity_23:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_23n
Identity_24IdentityRestoreV2:tensors:24"/device:CPU:0*
T0*
_output_shapes
:2
Identity_24?
AssignVariableOp_24AssignVariableOp#assignvariableop_24_dense_84_kernelIdentity_24:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_24n
Identity_25IdentityRestoreV2:tensors:25"/device:CPU:0*
T0*
_output_shapes
:2
Identity_25?
AssignVariableOp_25AssignVariableOp!assignvariableop_25_dense_84_biasIdentity_25:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_25n
Identity_26IdentityRestoreV2:tensors:26"/device:CPU:0*
T0*
_output_shapes
:2
Identity_26?
AssignVariableOp_26AssignVariableOp1assignvariableop_26_batch_normalization_172_gammaIdentity_26:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_26n
Identity_27IdentityRestoreV2:tensors:27"/device:CPU:0*
T0*
_output_shapes
:2
Identity_27?
AssignVariableOp_27AssignVariableOp0assignvariableop_27_batch_normalization_172_betaIdentity_27:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_27n
Identity_28IdentityRestoreV2:tensors:28"/device:CPU:0*
T0*
_output_shapes
:2
Identity_28?
AssignVariableOp_28AssignVariableOp7assignvariableop_28_batch_normalization_172_moving_meanIdentity_28:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_28n
Identity_29IdentityRestoreV2:tensors:29"/device:CPU:0*
T0*
_output_shapes
:2
Identity_29?
AssignVariableOp_29AssignVariableOp;assignvariableop_29_batch_normalization_172_moving_varianceIdentity_29:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_29n
Identity_30IdentityRestoreV2:tensors:30"/device:CPU:0*
T0*
_output_shapes
:2
Identity_30?
AssignVariableOp_30AssignVariableOp#assignvariableop_30_dense_85_kernelIdentity_30:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_30n
Identity_31IdentityRestoreV2:tensors:31"/device:CPU:0*
T0*
_output_shapes
:2
Identity_31?
AssignVariableOp_31AssignVariableOp!assignvariableop_31_dense_85_biasIdentity_31:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_31n
Identity_32IdentityRestoreV2:tensors:32"/device:CPU:0*
T0*
_output_shapes
:2
Identity_32?
AssignVariableOp_32AssignVariableOp1assignvariableop_32_batch_normalization_173_gammaIdentity_32:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_32n
Identity_33IdentityRestoreV2:tensors:33"/device:CPU:0*
T0*
_output_shapes
:2
Identity_33?
AssignVariableOp_33AssignVariableOp0assignvariableop_33_batch_normalization_173_betaIdentity_33:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_33n
Identity_34IdentityRestoreV2:tensors:34"/device:CPU:0*
T0*
_output_shapes
:2
Identity_34?
AssignVariableOp_34AssignVariableOp7assignvariableop_34_batch_normalization_173_moving_meanIdentity_34:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_34n
Identity_35IdentityRestoreV2:tensors:35"/device:CPU:0*
T0*
_output_shapes
:2
Identity_35?
AssignVariableOp_35AssignVariableOp;assignvariableop_35_batch_normalization_173_moving_varianceIdentity_35:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_35n
Identity_36IdentityRestoreV2:tensors:36"/device:CPU:0*
T0*
_output_shapes
:2
Identity_36?
AssignVariableOp_36AssignVariableOp#assignvariableop_36_dense_86_kernelIdentity_36:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_36n
Identity_37IdentityRestoreV2:tensors:37"/device:CPU:0*
T0*
_output_shapes
:2
Identity_37?
AssignVariableOp_37AssignVariableOp!assignvariableop_37_dense_86_biasIdentity_37:output:0"/device:CPU:0*
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
AssignVariableOp_47AssignVariableOp,assignvariableop_47_adam_conv2d_112_kernel_mIdentity_47:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_47n
Identity_48IdentityRestoreV2:tensors:48"/device:CPU:0*
T0*
_output_shapes
:2
Identity_48?
AssignVariableOp_48AssignVariableOp*assignvariableop_48_adam_conv2d_112_bias_mIdentity_48:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_48n
Identity_49IdentityRestoreV2:tensors:49"/device:CPU:0*
T0*
_output_shapes
:2
Identity_49?
AssignVariableOp_49AssignVariableOp8assignvariableop_49_adam_batch_normalization_168_gamma_mIdentity_49:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_49n
Identity_50IdentityRestoreV2:tensors:50"/device:CPU:0*
T0*
_output_shapes
:2
Identity_50?
AssignVariableOp_50AssignVariableOp7assignvariableop_50_adam_batch_normalization_168_beta_mIdentity_50:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_50n
Identity_51IdentityRestoreV2:tensors:51"/device:CPU:0*
T0*
_output_shapes
:2
Identity_51?
AssignVariableOp_51AssignVariableOp,assignvariableop_51_adam_conv2d_113_kernel_mIdentity_51:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_51n
Identity_52IdentityRestoreV2:tensors:52"/device:CPU:0*
T0*
_output_shapes
:2
Identity_52?
AssignVariableOp_52AssignVariableOp*assignvariableop_52_adam_conv2d_113_bias_mIdentity_52:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_52n
Identity_53IdentityRestoreV2:tensors:53"/device:CPU:0*
T0*
_output_shapes
:2
Identity_53?
AssignVariableOp_53AssignVariableOp8assignvariableop_53_adam_batch_normalization_169_gamma_mIdentity_53:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_53n
Identity_54IdentityRestoreV2:tensors:54"/device:CPU:0*
T0*
_output_shapes
:2
Identity_54?
AssignVariableOp_54AssignVariableOp7assignvariableop_54_adam_batch_normalization_169_beta_mIdentity_54:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_54n
Identity_55IdentityRestoreV2:tensors:55"/device:CPU:0*
T0*
_output_shapes
:2
Identity_55?
AssignVariableOp_55AssignVariableOp,assignvariableop_55_adam_conv2d_114_kernel_mIdentity_55:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_55n
Identity_56IdentityRestoreV2:tensors:56"/device:CPU:0*
T0*
_output_shapes
:2
Identity_56?
AssignVariableOp_56AssignVariableOp*assignvariableop_56_adam_conv2d_114_bias_mIdentity_56:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_56n
Identity_57IdentityRestoreV2:tensors:57"/device:CPU:0*
T0*
_output_shapes
:2
Identity_57?
AssignVariableOp_57AssignVariableOp8assignvariableop_57_adam_batch_normalization_170_gamma_mIdentity_57:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_57n
Identity_58IdentityRestoreV2:tensors:58"/device:CPU:0*
T0*
_output_shapes
:2
Identity_58?
AssignVariableOp_58AssignVariableOp7assignvariableop_58_adam_batch_normalization_170_beta_mIdentity_58:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_58n
Identity_59IdentityRestoreV2:tensors:59"/device:CPU:0*
T0*
_output_shapes
:2
Identity_59?
AssignVariableOp_59AssignVariableOp,assignvariableop_59_adam_conv2d_115_kernel_mIdentity_59:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_59n
Identity_60IdentityRestoreV2:tensors:60"/device:CPU:0*
T0*
_output_shapes
:2
Identity_60?
AssignVariableOp_60AssignVariableOp*assignvariableop_60_adam_conv2d_115_bias_mIdentity_60:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_60n
Identity_61IdentityRestoreV2:tensors:61"/device:CPU:0*
T0*
_output_shapes
:2
Identity_61?
AssignVariableOp_61AssignVariableOp8assignvariableop_61_adam_batch_normalization_171_gamma_mIdentity_61:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_61n
Identity_62IdentityRestoreV2:tensors:62"/device:CPU:0*
T0*
_output_shapes
:2
Identity_62?
AssignVariableOp_62AssignVariableOp7assignvariableop_62_adam_batch_normalization_171_beta_mIdentity_62:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_62n
Identity_63IdentityRestoreV2:tensors:63"/device:CPU:0*
T0*
_output_shapes
:2
Identity_63?
AssignVariableOp_63AssignVariableOp*assignvariableop_63_adam_dense_84_kernel_mIdentity_63:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_63n
Identity_64IdentityRestoreV2:tensors:64"/device:CPU:0*
T0*
_output_shapes
:2
Identity_64?
AssignVariableOp_64AssignVariableOp(assignvariableop_64_adam_dense_84_bias_mIdentity_64:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_64n
Identity_65IdentityRestoreV2:tensors:65"/device:CPU:0*
T0*
_output_shapes
:2
Identity_65?
AssignVariableOp_65AssignVariableOp8assignvariableop_65_adam_batch_normalization_172_gamma_mIdentity_65:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_65n
Identity_66IdentityRestoreV2:tensors:66"/device:CPU:0*
T0*
_output_shapes
:2
Identity_66?
AssignVariableOp_66AssignVariableOp7assignvariableop_66_adam_batch_normalization_172_beta_mIdentity_66:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_66n
Identity_67IdentityRestoreV2:tensors:67"/device:CPU:0*
T0*
_output_shapes
:2
Identity_67?
AssignVariableOp_67AssignVariableOp*assignvariableop_67_adam_dense_85_kernel_mIdentity_67:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_67n
Identity_68IdentityRestoreV2:tensors:68"/device:CPU:0*
T0*
_output_shapes
:2
Identity_68?
AssignVariableOp_68AssignVariableOp(assignvariableop_68_adam_dense_85_bias_mIdentity_68:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_68n
Identity_69IdentityRestoreV2:tensors:69"/device:CPU:0*
T0*
_output_shapes
:2
Identity_69?
AssignVariableOp_69AssignVariableOp8assignvariableop_69_adam_batch_normalization_173_gamma_mIdentity_69:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_69n
Identity_70IdentityRestoreV2:tensors:70"/device:CPU:0*
T0*
_output_shapes
:2
Identity_70?
AssignVariableOp_70AssignVariableOp7assignvariableop_70_adam_batch_normalization_173_beta_mIdentity_70:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_70n
Identity_71IdentityRestoreV2:tensors:71"/device:CPU:0*
T0*
_output_shapes
:2
Identity_71?
AssignVariableOp_71AssignVariableOp*assignvariableop_71_adam_dense_86_kernel_mIdentity_71:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_71n
Identity_72IdentityRestoreV2:tensors:72"/device:CPU:0*
T0*
_output_shapes
:2
Identity_72?
AssignVariableOp_72AssignVariableOp(assignvariableop_72_adam_dense_86_bias_mIdentity_72:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_72n
Identity_73IdentityRestoreV2:tensors:73"/device:CPU:0*
T0*
_output_shapes
:2
Identity_73?
AssignVariableOp_73AssignVariableOp,assignvariableop_73_adam_conv2d_112_kernel_vIdentity_73:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_73n
Identity_74IdentityRestoreV2:tensors:74"/device:CPU:0*
T0*
_output_shapes
:2
Identity_74?
AssignVariableOp_74AssignVariableOp*assignvariableop_74_adam_conv2d_112_bias_vIdentity_74:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_74n
Identity_75IdentityRestoreV2:tensors:75"/device:CPU:0*
T0*
_output_shapes
:2
Identity_75?
AssignVariableOp_75AssignVariableOp8assignvariableop_75_adam_batch_normalization_168_gamma_vIdentity_75:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_75n
Identity_76IdentityRestoreV2:tensors:76"/device:CPU:0*
T0*
_output_shapes
:2
Identity_76?
AssignVariableOp_76AssignVariableOp7assignvariableop_76_adam_batch_normalization_168_beta_vIdentity_76:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_76n
Identity_77IdentityRestoreV2:tensors:77"/device:CPU:0*
T0*
_output_shapes
:2
Identity_77?
AssignVariableOp_77AssignVariableOp,assignvariableop_77_adam_conv2d_113_kernel_vIdentity_77:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_77n
Identity_78IdentityRestoreV2:tensors:78"/device:CPU:0*
T0*
_output_shapes
:2
Identity_78?
AssignVariableOp_78AssignVariableOp*assignvariableop_78_adam_conv2d_113_bias_vIdentity_78:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_78n
Identity_79IdentityRestoreV2:tensors:79"/device:CPU:0*
T0*
_output_shapes
:2
Identity_79?
AssignVariableOp_79AssignVariableOp8assignvariableop_79_adam_batch_normalization_169_gamma_vIdentity_79:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_79n
Identity_80IdentityRestoreV2:tensors:80"/device:CPU:0*
T0*
_output_shapes
:2
Identity_80?
AssignVariableOp_80AssignVariableOp7assignvariableop_80_adam_batch_normalization_169_beta_vIdentity_80:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_80n
Identity_81IdentityRestoreV2:tensors:81"/device:CPU:0*
T0*
_output_shapes
:2
Identity_81?
AssignVariableOp_81AssignVariableOp,assignvariableop_81_adam_conv2d_114_kernel_vIdentity_81:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_81n
Identity_82IdentityRestoreV2:tensors:82"/device:CPU:0*
T0*
_output_shapes
:2
Identity_82?
AssignVariableOp_82AssignVariableOp*assignvariableop_82_adam_conv2d_114_bias_vIdentity_82:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_82n
Identity_83IdentityRestoreV2:tensors:83"/device:CPU:0*
T0*
_output_shapes
:2
Identity_83?
AssignVariableOp_83AssignVariableOp8assignvariableop_83_adam_batch_normalization_170_gamma_vIdentity_83:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_83n
Identity_84IdentityRestoreV2:tensors:84"/device:CPU:0*
T0*
_output_shapes
:2
Identity_84?
AssignVariableOp_84AssignVariableOp7assignvariableop_84_adam_batch_normalization_170_beta_vIdentity_84:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_84n
Identity_85IdentityRestoreV2:tensors:85"/device:CPU:0*
T0*
_output_shapes
:2
Identity_85?
AssignVariableOp_85AssignVariableOp,assignvariableop_85_adam_conv2d_115_kernel_vIdentity_85:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_85n
Identity_86IdentityRestoreV2:tensors:86"/device:CPU:0*
T0*
_output_shapes
:2
Identity_86?
AssignVariableOp_86AssignVariableOp*assignvariableop_86_adam_conv2d_115_bias_vIdentity_86:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_86n
Identity_87IdentityRestoreV2:tensors:87"/device:CPU:0*
T0*
_output_shapes
:2
Identity_87?
AssignVariableOp_87AssignVariableOp8assignvariableop_87_adam_batch_normalization_171_gamma_vIdentity_87:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_87n
Identity_88IdentityRestoreV2:tensors:88"/device:CPU:0*
T0*
_output_shapes
:2
Identity_88?
AssignVariableOp_88AssignVariableOp7assignvariableop_88_adam_batch_normalization_171_beta_vIdentity_88:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_88n
Identity_89IdentityRestoreV2:tensors:89"/device:CPU:0*
T0*
_output_shapes
:2
Identity_89?
AssignVariableOp_89AssignVariableOp*assignvariableop_89_adam_dense_84_kernel_vIdentity_89:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_89n
Identity_90IdentityRestoreV2:tensors:90"/device:CPU:0*
T0*
_output_shapes
:2
Identity_90?
AssignVariableOp_90AssignVariableOp(assignvariableop_90_adam_dense_84_bias_vIdentity_90:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_90n
Identity_91IdentityRestoreV2:tensors:91"/device:CPU:0*
T0*
_output_shapes
:2
Identity_91?
AssignVariableOp_91AssignVariableOp8assignvariableop_91_adam_batch_normalization_172_gamma_vIdentity_91:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_91n
Identity_92IdentityRestoreV2:tensors:92"/device:CPU:0*
T0*
_output_shapes
:2
Identity_92?
AssignVariableOp_92AssignVariableOp7assignvariableop_92_adam_batch_normalization_172_beta_vIdentity_92:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_92n
Identity_93IdentityRestoreV2:tensors:93"/device:CPU:0*
T0*
_output_shapes
:2
Identity_93?
AssignVariableOp_93AssignVariableOp*assignvariableop_93_adam_dense_85_kernel_vIdentity_93:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_93n
Identity_94IdentityRestoreV2:tensors:94"/device:CPU:0*
T0*
_output_shapes
:2
Identity_94?
AssignVariableOp_94AssignVariableOp(assignvariableop_94_adam_dense_85_bias_vIdentity_94:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_94n
Identity_95IdentityRestoreV2:tensors:95"/device:CPU:0*
T0*
_output_shapes
:2
Identity_95?
AssignVariableOp_95AssignVariableOp8assignvariableop_95_adam_batch_normalization_173_gamma_vIdentity_95:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_95n
Identity_96IdentityRestoreV2:tensors:96"/device:CPU:0*
T0*
_output_shapes
:2
Identity_96?
AssignVariableOp_96AssignVariableOp7assignvariableop_96_adam_batch_normalization_173_beta_vIdentity_96:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_96n
Identity_97IdentityRestoreV2:tensors:97"/device:CPU:0*
T0*
_output_shapes
:2
Identity_97?
AssignVariableOp_97AssignVariableOp*assignvariableop_97_adam_dense_86_kernel_vIdentity_97:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_97n
Identity_98IdentityRestoreV2:tensors:98"/device:CPU:0*
T0*
_output_shapes
:2
Identity_98?
AssignVariableOp_98AssignVariableOp(assignvariableop_98_adam_dense_86_bias_vIdentity_98:output:0"/device:CPU:0*
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
?
e
,__inference_dropout_86_layer_call_fn_1147109

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
GPU2*0J 8? *P
fKRI
G__inference_dropout_86_layer_call_and_return_conditional_losses_11450052
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
?
?
,__inference_conv2d_113_layer_call_fn_1146337

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
GPU2*0J 8? *P
fKRI
G__inference_conv2d_113_layer_call_and_return_conditional_losses_11444292
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
?
g
K__inference_activation_201_layer_call_and_return_conditional_losses_1144950

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
?
?
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_1146699

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
?
?
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_1143645

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
?
N
2__inference_max_pooling2d_56_layer_call_fn_1143803

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
GPU2*0J 8? *V
fQRO
M__inference_max_pooling2d_56_layer_call_and_return_conditional_losses_11437972
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
?
f
G__inference_dropout_86_layer_call_and_return_conditional_losses_1147099

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
?
c
G__inference_flatten_57_layer_call_and_return_conditional_losses_1146971

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
??
?
J__inference_sequential_28_layer_call_and_return_conditional_losses_1145999

inputs-
)conv2d_112_conv2d_readvariableop_resource.
*conv2d_112_biasadd_readvariableop_resource3
/batch_normalization_168_readvariableop_resource5
1batch_normalization_168_readvariableop_1_resourceD
@batch_normalization_168_fusedbatchnormv3_readvariableop_resourceF
Bbatch_normalization_168_fusedbatchnormv3_readvariableop_1_resource-
)conv2d_113_conv2d_readvariableop_resource.
*conv2d_113_biasadd_readvariableop_resource3
/batch_normalization_169_readvariableop_resource5
1batch_normalization_169_readvariableop_1_resourceD
@batch_normalization_169_fusedbatchnormv3_readvariableop_resourceF
Bbatch_normalization_169_fusedbatchnormv3_readvariableop_1_resource-
)conv2d_114_conv2d_readvariableop_resource.
*conv2d_114_biasadd_readvariableop_resource3
/batch_normalization_170_readvariableop_resource5
1batch_normalization_170_readvariableop_1_resourceD
@batch_normalization_170_fusedbatchnormv3_readvariableop_resourceF
Bbatch_normalization_170_fusedbatchnormv3_readvariableop_1_resource-
)conv2d_115_conv2d_readvariableop_resource.
*conv2d_115_biasadd_readvariableop_resource3
/batch_normalization_171_readvariableop_resource5
1batch_normalization_171_readvariableop_1_resourceD
@batch_normalization_171_fusedbatchnormv3_readvariableop_resourceF
Bbatch_normalization_171_fusedbatchnormv3_readvariableop_1_resource+
'dense_84_matmul_readvariableop_resource,
(dense_84_biasadd_readvariableop_resource=
9batch_normalization_172_batchnorm_readvariableop_resourceA
=batch_normalization_172_batchnorm_mul_readvariableop_resource?
;batch_normalization_172_batchnorm_readvariableop_1_resource?
;batch_normalization_172_batchnorm_readvariableop_2_resource+
'dense_85_matmul_readvariableop_resource,
(dense_85_biasadd_readvariableop_resource=
9batch_normalization_173_batchnorm_readvariableop_resourceA
=batch_normalization_173_batchnorm_mul_readvariableop_resource?
;batch_normalization_173_batchnorm_readvariableop_1_resource?
;batch_normalization_173_batchnorm_readvariableop_2_resource+
'dense_86_matmul_readvariableop_resource,
(dense_86_biasadd_readvariableop_resource
identity??7batch_normalization_168/FusedBatchNormV3/ReadVariableOp?9batch_normalization_168/FusedBatchNormV3/ReadVariableOp_1?&batch_normalization_168/ReadVariableOp?(batch_normalization_168/ReadVariableOp_1?7batch_normalization_169/FusedBatchNormV3/ReadVariableOp?9batch_normalization_169/FusedBatchNormV3/ReadVariableOp_1?&batch_normalization_169/ReadVariableOp?(batch_normalization_169/ReadVariableOp_1?7batch_normalization_170/FusedBatchNormV3/ReadVariableOp?9batch_normalization_170/FusedBatchNormV3/ReadVariableOp_1?&batch_normalization_170/ReadVariableOp?(batch_normalization_170/ReadVariableOp_1?7batch_normalization_171/FusedBatchNormV3/ReadVariableOp?9batch_normalization_171/FusedBatchNormV3/ReadVariableOp_1?&batch_normalization_171/ReadVariableOp?(batch_normalization_171/ReadVariableOp_1?0batch_normalization_172/batchnorm/ReadVariableOp?2batch_normalization_172/batchnorm/ReadVariableOp_1?2batch_normalization_172/batchnorm/ReadVariableOp_2?4batch_normalization_172/batchnorm/mul/ReadVariableOp?0batch_normalization_173/batchnorm/ReadVariableOp?2batch_normalization_173/batchnorm/ReadVariableOp_1?2batch_normalization_173/batchnorm/ReadVariableOp_2?4batch_normalization_173/batchnorm/mul/ReadVariableOp?!conv2d_112/BiasAdd/ReadVariableOp? conv2d_112/Conv2D/ReadVariableOp?!conv2d_113/BiasAdd/ReadVariableOp? conv2d_113/Conv2D/ReadVariableOp?!conv2d_114/BiasAdd/ReadVariableOp? conv2d_114/Conv2D/ReadVariableOp?!conv2d_115/BiasAdd/ReadVariableOp? conv2d_115/Conv2D/ReadVariableOp?dense_84/BiasAdd/ReadVariableOp?dense_84/MatMul/ReadVariableOp?dense_85/BiasAdd/ReadVariableOp?dense_85/MatMul/ReadVariableOp?dense_86/BiasAdd/ReadVariableOp?dense_86/MatMul/ReadVariableOp?
 conv2d_112/Conv2D/ReadVariableOpReadVariableOp)conv2d_112_conv2d_readvariableop_resource*&
_output_shapes
:<*
dtype02"
 conv2d_112/Conv2D/ReadVariableOp?
conv2d_112/Conv2DConv2Dinputs(conv2d_112/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????<<<*
paddingVALID*
strides
2
conv2d_112/Conv2D?
!conv2d_112/BiasAdd/ReadVariableOpReadVariableOp*conv2d_112_biasadd_readvariableop_resource*
_output_shapes
:<*
dtype02#
!conv2d_112/BiasAdd/ReadVariableOp?
conv2d_112/BiasAddBiasAddconv2d_112/Conv2D:output:0)conv2d_112/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????<<<2
conv2d_112/BiasAdd?
activation_196/ReluReluconv2d_112/BiasAdd:output:0*
T0*/
_output_shapes
:?????????<<<2
activation_196/Relu?
&batch_normalization_168/ReadVariableOpReadVariableOp/batch_normalization_168_readvariableop_resource*
_output_shapes
:<*
dtype02(
&batch_normalization_168/ReadVariableOp?
(batch_normalization_168/ReadVariableOp_1ReadVariableOp1batch_normalization_168_readvariableop_1_resource*
_output_shapes
:<*
dtype02*
(batch_normalization_168/ReadVariableOp_1?
7batch_normalization_168/FusedBatchNormV3/ReadVariableOpReadVariableOp@batch_normalization_168_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype029
7batch_normalization_168/FusedBatchNormV3/ReadVariableOp?
9batch_normalization_168/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpBbatch_normalization_168_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02;
9batch_normalization_168/FusedBatchNormV3/ReadVariableOp_1?
(batch_normalization_168/FusedBatchNormV3FusedBatchNormV3!activation_196/Relu:activations:0.batch_normalization_168/ReadVariableOp:value:00batch_normalization_168/ReadVariableOp_1:value:0?batch_normalization_168/FusedBatchNormV3/ReadVariableOp:value:0Abatch_normalization_168/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????<<<:<:<:<:<:*
epsilon%o?:*
is_training( 2*
(batch_normalization_168/FusedBatchNormV3?
 conv2d_113/Conv2D/ReadVariableOpReadVariableOp)conv2d_113_conv2d_readvariableop_resource*&
_output_shapes
:<<*
dtype02"
 conv2d_113/Conv2D/ReadVariableOp?
conv2d_113/Conv2DConv2D,batch_normalization_168/FusedBatchNormV3:y:0(conv2d_113/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????88<*
paddingVALID*
strides
2
conv2d_113/Conv2D?
!conv2d_113/BiasAdd/ReadVariableOpReadVariableOp*conv2d_113_biasadd_readvariableop_resource*
_output_shapes
:<*
dtype02#
!conv2d_113/BiasAdd/ReadVariableOp?
conv2d_113/BiasAddBiasAddconv2d_113/Conv2D:output:0)conv2d_113/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????88<2
conv2d_113/BiasAdd?
activation_197/ReluReluconv2d_113/BiasAdd:output:0*
T0*/
_output_shapes
:?????????88<2
activation_197/Relu?
&batch_normalization_169/ReadVariableOpReadVariableOp/batch_normalization_169_readvariableop_resource*
_output_shapes
:<*
dtype02(
&batch_normalization_169/ReadVariableOp?
(batch_normalization_169/ReadVariableOp_1ReadVariableOp1batch_normalization_169_readvariableop_1_resource*
_output_shapes
:<*
dtype02*
(batch_normalization_169/ReadVariableOp_1?
7batch_normalization_169/FusedBatchNormV3/ReadVariableOpReadVariableOp@batch_normalization_169_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:<*
dtype029
7batch_normalization_169/FusedBatchNormV3/ReadVariableOp?
9batch_normalization_169/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpBbatch_normalization_169_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:<*
dtype02;
9batch_normalization_169/FusedBatchNormV3/ReadVariableOp_1?
(batch_normalization_169/FusedBatchNormV3FusedBatchNormV3!activation_197/Relu:activations:0.batch_normalization_169/ReadVariableOp:value:00batch_normalization_169/ReadVariableOp_1:value:0?batch_normalization_169/FusedBatchNormV3/ReadVariableOp:value:0Abatch_normalization_169/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????88<:<:<:<:<:*
epsilon%o?:*
is_training( 2*
(batch_normalization_169/FusedBatchNormV3?
max_pooling2d_56/MaxPoolMaxPool,batch_normalization_169/FusedBatchNormV3:y:0*/
_output_shapes
:?????????<*
ksize
*
paddingVALID*
strides
2
max_pooling2d_56/MaxPool?
 conv2d_114/Conv2D/ReadVariableOpReadVariableOp)conv2d_114_conv2d_readvariableop_resource*&
_output_shapes
:<*
dtype02"
 conv2d_114/Conv2D/ReadVariableOp?
conv2d_114/Conv2DConv2D!max_pooling2d_56/MaxPool:output:0(conv2d_114/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2
conv2d_114/Conv2D?
!conv2d_114/BiasAdd/ReadVariableOpReadVariableOp*conv2d_114_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02#
!conv2d_114/BiasAdd/ReadVariableOp?
conv2d_114/BiasAddBiasAddconv2d_114/Conv2D:output:0)conv2d_114/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????2
conv2d_114/BiasAdd?
activation_198/ReluReluconv2d_114/BiasAdd:output:0*
T0*/
_output_shapes
:?????????2
activation_198/Relu?
&batch_normalization_170/ReadVariableOpReadVariableOp/batch_normalization_170_readvariableop_resource*
_output_shapes
:*
dtype02(
&batch_normalization_170/ReadVariableOp?
(batch_normalization_170/ReadVariableOp_1ReadVariableOp1batch_normalization_170_readvariableop_1_resource*
_output_shapes
:*
dtype02*
(batch_normalization_170/ReadVariableOp_1?
7batch_normalization_170/FusedBatchNormV3/ReadVariableOpReadVariableOp@batch_normalization_170_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype029
7batch_normalization_170/FusedBatchNormV3/ReadVariableOp?
9batch_normalization_170/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpBbatch_normalization_170_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02;
9batch_normalization_170/FusedBatchNormV3/ReadVariableOp_1?
(batch_normalization_170/FusedBatchNormV3FusedBatchNormV3!activation_198/Relu:activations:0.batch_normalization_170/ReadVariableOp:value:00batch_normalization_170/ReadVariableOp_1:value:0?batch_normalization_170/FusedBatchNormV3/ReadVariableOp:value:0Abatch_normalization_170/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
is_training( 2*
(batch_normalization_170/FusedBatchNormV3?
 conv2d_115/Conv2D/ReadVariableOpReadVariableOp)conv2d_115_conv2d_readvariableop_resource*&
_output_shapes
:*
dtype02"
 conv2d_115/Conv2D/ReadVariableOp?
conv2d_115/Conv2DConv2D,batch_normalization_170/FusedBatchNormV3:y:0(conv2d_115/Conv2D/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2
conv2d_115/Conv2D?
!conv2d_115/BiasAdd/ReadVariableOpReadVariableOp*conv2d_115_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02#
!conv2d_115/BiasAdd/ReadVariableOp?
conv2d_115/BiasAddBiasAddconv2d_115/Conv2D:output:0)conv2d_115/BiasAdd/ReadVariableOp:value:0*
T0*/
_output_shapes
:?????????2
conv2d_115/BiasAdd?
activation_199/ReluReluconv2d_115/BiasAdd:output:0*
T0*/
_output_shapes
:?????????2
activation_199/Relu?
&batch_normalization_171/ReadVariableOpReadVariableOp/batch_normalization_171_readvariableop_resource*
_output_shapes
:*
dtype02(
&batch_normalization_171/ReadVariableOp?
(batch_normalization_171/ReadVariableOp_1ReadVariableOp1batch_normalization_171_readvariableop_1_resource*
_output_shapes
:*
dtype02*
(batch_normalization_171/ReadVariableOp_1?
7batch_normalization_171/FusedBatchNormV3/ReadVariableOpReadVariableOp@batch_normalization_171_fusedbatchnormv3_readvariableop_resource*
_output_shapes
:*
dtype029
7batch_normalization_171/FusedBatchNormV3/ReadVariableOp?
9batch_normalization_171/FusedBatchNormV3/ReadVariableOp_1ReadVariableOpBbatch_normalization_171_fusedbatchnormv3_readvariableop_1_resource*
_output_shapes
:*
dtype02;
9batch_normalization_171/FusedBatchNormV3/ReadVariableOp_1?
(batch_normalization_171/FusedBatchNormV3FusedBatchNormV3!activation_199/Relu:activations:0.batch_normalization_171/ReadVariableOp:value:00batch_normalization_171/ReadVariableOp_1:value:0?batch_normalization_171/FusedBatchNormV3/ReadVariableOp:value:0Abatch_normalization_171/FusedBatchNormV3/ReadVariableOp_1:value:0*
T0*
U0*K
_output_shapes9
7:?????????:::::*
epsilon%o?:*
is_training( 2*
(batch_normalization_171/FusedBatchNormV3?
max_pooling2d_57/MaxPoolMaxPool,batch_normalization_171/FusedBatchNormV3:y:0*/
_output_shapes
:?????????*
ksize
*
paddingVALID*
strides
2
max_pooling2d_57/MaxPool?
dropout_84/IdentityIdentity!max_pooling2d_57/MaxPool:output:0*
T0*/
_output_shapes
:?????????2
dropout_84/Identityu
flatten_56/ConstConst*
_output_shapes
:*
dtype0*
valueB"?????  2
flatten_56/Const?
flatten_56/ReshapeReshapedropout_84/Identity:output:0flatten_56/Const:output:0*
T0*(
_output_shapes
:??????????!2
flatten_56/Reshape?
dense_84/MatMul/ReadVariableOpReadVariableOp'dense_84_matmul_readvariableop_resource* 
_output_shapes
:
?!?*
dtype02 
dense_84/MatMul/ReadVariableOp?
dense_84/MatMulMatMulflatten_56/Reshape:output:0&dense_84/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
dense_84/MatMul?
dense_84/BiasAdd/ReadVariableOpReadVariableOp(dense_84_biasadd_readvariableop_resource*
_output_shapes	
:?*
dtype02!
dense_84/BiasAdd/ReadVariableOp?
dense_84/BiasAddBiasAdddense_84/MatMul:product:0'dense_84/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
dense_84/BiasAdd?
activation_200/ReluReludense_84/BiasAdd:output:0*
T0*(
_output_shapes
:??????????2
activation_200/Relu?
0batch_normalization_172/batchnorm/ReadVariableOpReadVariableOp9batch_normalization_172_batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype022
0batch_normalization_172/batchnorm/ReadVariableOp?
'batch_normalization_172/batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:2)
'batch_normalization_172/batchnorm/add/y?
%batch_normalization_172/batchnorm/addAddV28batch_normalization_172/batchnorm/ReadVariableOp:value:00batch_normalization_172/batchnorm/add/y:output:0*
T0*
_output_shapes	
:?2'
%batch_normalization_172/batchnorm/add?
'batch_normalization_172/batchnorm/RsqrtRsqrt)batch_normalization_172/batchnorm/add:z:0*
T0*
_output_shapes	
:?2)
'batch_normalization_172/batchnorm/Rsqrt?
4batch_normalization_172/batchnorm/mul/ReadVariableOpReadVariableOp=batch_normalization_172_batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype026
4batch_normalization_172/batchnorm/mul/ReadVariableOp?
%batch_normalization_172/batchnorm/mulMul+batch_normalization_172/batchnorm/Rsqrt:y:0<batch_normalization_172/batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?2'
%batch_normalization_172/batchnorm/mul?
'batch_normalization_172/batchnorm/mul_1Mul!activation_200/Relu:activations:0)batch_normalization_172/batchnorm/mul:z:0*
T0*(
_output_shapes
:??????????2)
'batch_normalization_172/batchnorm/mul_1?
2batch_normalization_172/batchnorm/ReadVariableOp_1ReadVariableOp;batch_normalization_172_batchnorm_readvariableop_1_resource*
_output_shapes	
:?*
dtype024
2batch_normalization_172/batchnorm/ReadVariableOp_1?
'batch_normalization_172/batchnorm/mul_2Mul:batch_normalization_172/batchnorm/ReadVariableOp_1:value:0)batch_normalization_172/batchnorm/mul:z:0*
T0*
_output_shapes	
:?2)
'batch_normalization_172/batchnorm/mul_2?
2batch_normalization_172/batchnorm/ReadVariableOp_2ReadVariableOp;batch_normalization_172_batchnorm_readvariableop_2_resource*
_output_shapes	
:?*
dtype024
2batch_normalization_172/batchnorm/ReadVariableOp_2?
%batch_normalization_172/batchnorm/subSub:batch_normalization_172/batchnorm/ReadVariableOp_2:value:0+batch_normalization_172/batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?2'
%batch_normalization_172/batchnorm/sub?
'batch_normalization_172/batchnorm/add_1AddV2+batch_normalization_172/batchnorm/mul_1:z:0)batch_normalization_172/batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????2)
'batch_normalization_172/batchnorm/add_1?
dropout_85/IdentityIdentity+batch_normalization_172/batchnorm/add_1:z:0*
T0*(
_output_shapes
:??????????2
dropout_85/Identityu
flatten_57/ConstConst*
_output_shapes
:*
dtype0*
valueB"????,  2
flatten_57/Const?
flatten_57/ReshapeReshapedropout_85/Identity:output:0flatten_57/Const:output:0*
T0*(
_output_shapes
:??????????2
flatten_57/Reshape?
dense_85/MatMul/ReadVariableOpReadVariableOp'dense_85_matmul_readvariableop_resource* 
_output_shapes
:
??*
dtype02 
dense_85/MatMul/ReadVariableOp?
dense_85/MatMulMatMulflatten_57/Reshape:output:0&dense_85/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
dense_85/MatMul?
dense_85/BiasAdd/ReadVariableOpReadVariableOp(dense_85_biasadd_readvariableop_resource*
_output_shapes	
:?*
dtype02!
dense_85/BiasAdd/ReadVariableOp?
dense_85/BiasAddBiasAdddense_85/MatMul:product:0'dense_85/BiasAdd/ReadVariableOp:value:0*
T0*(
_output_shapes
:??????????2
dense_85/BiasAdd?
activation_201/ReluReludense_85/BiasAdd:output:0*
T0*(
_output_shapes
:??????????2
activation_201/Relu?
0batch_normalization_173/batchnorm/ReadVariableOpReadVariableOp9batch_normalization_173_batchnorm_readvariableop_resource*
_output_shapes	
:?*
dtype022
0batch_normalization_173/batchnorm/ReadVariableOp?
'batch_normalization_173/batchnorm/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *o?:2)
'batch_normalization_173/batchnorm/add/y?
%batch_normalization_173/batchnorm/addAddV28batch_normalization_173/batchnorm/ReadVariableOp:value:00batch_normalization_173/batchnorm/add/y:output:0*
T0*
_output_shapes	
:?2'
%batch_normalization_173/batchnorm/add?
'batch_normalization_173/batchnorm/RsqrtRsqrt)batch_normalization_173/batchnorm/add:z:0*
T0*
_output_shapes	
:?2)
'batch_normalization_173/batchnorm/Rsqrt?
4batch_normalization_173/batchnorm/mul/ReadVariableOpReadVariableOp=batch_normalization_173_batchnorm_mul_readvariableop_resource*
_output_shapes	
:?*
dtype026
4batch_normalization_173/batchnorm/mul/ReadVariableOp?
%batch_normalization_173/batchnorm/mulMul+batch_normalization_173/batchnorm/Rsqrt:y:0<batch_normalization_173/batchnorm/mul/ReadVariableOp:value:0*
T0*
_output_shapes	
:?2'
%batch_normalization_173/batchnorm/mul?
'batch_normalization_173/batchnorm/mul_1Mul!activation_201/Relu:activations:0)batch_normalization_173/batchnorm/mul:z:0*
T0*(
_output_shapes
:??????????2)
'batch_normalization_173/batchnorm/mul_1?
2batch_normalization_173/batchnorm/ReadVariableOp_1ReadVariableOp;batch_normalization_173_batchnorm_readvariableop_1_resource*
_output_shapes	
:?*
dtype024
2batch_normalization_173/batchnorm/ReadVariableOp_1?
'batch_normalization_173/batchnorm/mul_2Mul:batch_normalization_173/batchnorm/ReadVariableOp_1:value:0)batch_normalization_173/batchnorm/mul:z:0*
T0*
_output_shapes	
:?2)
'batch_normalization_173/batchnorm/mul_2?
2batch_normalization_173/batchnorm/ReadVariableOp_2ReadVariableOp;batch_normalization_173_batchnorm_readvariableop_2_resource*
_output_shapes	
:?*
dtype024
2batch_normalization_173/batchnorm/ReadVariableOp_2?
%batch_normalization_173/batchnorm/subSub:batch_normalization_173/batchnorm/ReadVariableOp_2:value:0+batch_normalization_173/batchnorm/mul_2:z:0*
T0*
_output_shapes	
:?2'
%batch_normalization_173/batchnorm/sub?
'batch_normalization_173/batchnorm/add_1AddV2+batch_normalization_173/batchnorm/mul_1:z:0)batch_normalization_173/batchnorm/sub:z:0*
T0*(
_output_shapes
:??????????2)
'batch_normalization_173/batchnorm/add_1?
dropout_86/IdentityIdentity+batch_normalization_173/batchnorm/add_1:z:0*
T0*(
_output_shapes
:??????????2
dropout_86/Identity?
dense_86/MatMul/ReadVariableOpReadVariableOp'dense_86_matmul_readvariableop_resource*
_output_shapes
:	?*
dtype02 
dense_86/MatMul/ReadVariableOp?
dense_86/MatMulMatMuldropout_86/Identity:output:0&dense_86/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
dense_86/MatMul?
dense_86/BiasAdd/ReadVariableOpReadVariableOp(dense_86_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02!
dense_86/BiasAdd/ReadVariableOp?
dense_86/BiasAddBiasAdddense_86/MatMul:product:0'dense_86/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
dense_86/BiasAdd?
activation_202/SoftmaxSoftmaxdense_86/BiasAdd:output:0*
T0*'
_output_shapes
:?????????2
activation_202/Softmax?
IdentityIdentity activation_202/Softmax:softmax:08^batch_normalization_168/FusedBatchNormV3/ReadVariableOp:^batch_normalization_168/FusedBatchNormV3/ReadVariableOp_1'^batch_normalization_168/ReadVariableOp)^batch_normalization_168/ReadVariableOp_18^batch_normalization_169/FusedBatchNormV3/ReadVariableOp:^batch_normalization_169/FusedBatchNormV3/ReadVariableOp_1'^batch_normalization_169/ReadVariableOp)^batch_normalization_169/ReadVariableOp_18^batch_normalization_170/FusedBatchNormV3/ReadVariableOp:^batch_normalization_170/FusedBatchNormV3/ReadVariableOp_1'^batch_normalization_170/ReadVariableOp)^batch_normalization_170/ReadVariableOp_18^batch_normalization_171/FusedBatchNormV3/ReadVariableOp:^batch_normalization_171/FusedBatchNormV3/ReadVariableOp_1'^batch_normalization_171/ReadVariableOp)^batch_normalization_171/ReadVariableOp_11^batch_normalization_172/batchnorm/ReadVariableOp3^batch_normalization_172/batchnorm/ReadVariableOp_13^batch_normalization_172/batchnorm/ReadVariableOp_25^batch_normalization_172/batchnorm/mul/ReadVariableOp1^batch_normalization_173/batchnorm/ReadVariableOp3^batch_normalization_173/batchnorm/ReadVariableOp_13^batch_normalization_173/batchnorm/ReadVariableOp_25^batch_normalization_173/batchnorm/mul/ReadVariableOp"^conv2d_112/BiasAdd/ReadVariableOp!^conv2d_112/Conv2D/ReadVariableOp"^conv2d_113/BiasAdd/ReadVariableOp!^conv2d_113/Conv2D/ReadVariableOp"^conv2d_114/BiasAdd/ReadVariableOp!^conv2d_114/Conv2D/ReadVariableOp"^conv2d_115/BiasAdd/ReadVariableOp!^conv2d_115/Conv2D/ReadVariableOp ^dense_84/BiasAdd/ReadVariableOp^dense_84/MatMul/ReadVariableOp ^dense_85/BiasAdd/ReadVariableOp^dense_85/MatMul/ReadVariableOp ^dense_86/BiasAdd/ReadVariableOp^dense_86/MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*?
_input_shapes?
?:?????????@@::::::::::::::::::::::::::::::::::::::2r
7batch_normalization_168/FusedBatchNormV3/ReadVariableOp7batch_normalization_168/FusedBatchNormV3/ReadVariableOp2v
9batch_normalization_168/FusedBatchNormV3/ReadVariableOp_19batch_normalization_168/FusedBatchNormV3/ReadVariableOp_12P
&batch_normalization_168/ReadVariableOp&batch_normalization_168/ReadVariableOp2T
(batch_normalization_168/ReadVariableOp_1(batch_normalization_168/ReadVariableOp_12r
7batch_normalization_169/FusedBatchNormV3/ReadVariableOp7batch_normalization_169/FusedBatchNormV3/ReadVariableOp2v
9batch_normalization_169/FusedBatchNormV3/ReadVariableOp_19batch_normalization_169/FusedBatchNormV3/ReadVariableOp_12P
&batch_normalization_169/ReadVariableOp&batch_normalization_169/ReadVariableOp2T
(batch_normalization_169/ReadVariableOp_1(batch_normalization_169/ReadVariableOp_12r
7batch_normalization_170/FusedBatchNormV3/ReadVariableOp7batch_normalization_170/FusedBatchNormV3/ReadVariableOp2v
9batch_normalization_170/FusedBatchNormV3/ReadVariableOp_19batch_normalization_170/FusedBatchNormV3/ReadVariableOp_12P
&batch_normalization_170/ReadVariableOp&batch_normalization_170/ReadVariableOp2T
(batch_normalization_170/ReadVariableOp_1(batch_normalization_170/ReadVariableOp_12r
7batch_normalization_171/FusedBatchNormV3/ReadVariableOp7batch_normalization_171/FusedBatchNormV3/ReadVariableOp2v
9batch_normalization_171/FusedBatchNormV3/ReadVariableOp_19batch_normalization_171/FusedBatchNormV3/ReadVariableOp_12P
&batch_normalization_171/ReadVariableOp&batch_normalization_171/ReadVariableOp2T
(batch_normalization_171/ReadVariableOp_1(batch_normalization_171/ReadVariableOp_12d
0batch_normalization_172/batchnorm/ReadVariableOp0batch_normalization_172/batchnorm/ReadVariableOp2h
2batch_normalization_172/batchnorm/ReadVariableOp_12batch_normalization_172/batchnorm/ReadVariableOp_12h
2batch_normalization_172/batchnorm/ReadVariableOp_22batch_normalization_172/batchnorm/ReadVariableOp_22l
4batch_normalization_172/batchnorm/mul/ReadVariableOp4batch_normalization_172/batchnorm/mul/ReadVariableOp2d
0batch_normalization_173/batchnorm/ReadVariableOp0batch_normalization_173/batchnorm/ReadVariableOp2h
2batch_normalization_173/batchnorm/ReadVariableOp_12batch_normalization_173/batchnorm/ReadVariableOp_12h
2batch_normalization_173/batchnorm/ReadVariableOp_22batch_normalization_173/batchnorm/ReadVariableOp_22l
4batch_normalization_173/batchnorm/mul/ReadVariableOp4batch_normalization_173/batchnorm/mul/ReadVariableOp2F
!conv2d_112/BiasAdd/ReadVariableOp!conv2d_112/BiasAdd/ReadVariableOp2D
 conv2d_112/Conv2D/ReadVariableOp conv2d_112/Conv2D/ReadVariableOp2F
!conv2d_113/BiasAdd/ReadVariableOp!conv2d_113/BiasAdd/ReadVariableOp2D
 conv2d_113/Conv2D/ReadVariableOp conv2d_113/Conv2D/ReadVariableOp2F
!conv2d_114/BiasAdd/ReadVariableOp!conv2d_114/BiasAdd/ReadVariableOp2D
 conv2d_114/Conv2D/ReadVariableOp conv2d_114/Conv2D/ReadVariableOp2F
!conv2d_115/BiasAdd/ReadVariableOp!conv2d_115/BiasAdd/ReadVariableOp2D
 conv2d_115/Conv2D/ReadVariableOp conv2d_115/Conv2D/ReadVariableOp2B
dense_84/BiasAdd/ReadVariableOpdense_84/BiasAdd/ReadVariableOp2@
dense_84/MatMul/ReadVariableOpdense_84/MatMul/ReadVariableOp2B
dense_85/BiasAdd/ReadVariableOpdense_85/BiasAdd/ReadVariableOp2@
dense_85/MatMul/ReadVariableOpdense_85/MatMul/ReadVariableOp2B
dense_86/BiasAdd/ReadVariableOpdense_86/BiasAdd/ReadVariableOp2@
dense_86/MatMul/ReadVariableOpdense_86/MatMul/ReadVariableOp:W S
/
_output_shapes
:?????????@@
 
_user_specified_nameinputs
?	
?
G__inference_conv2d_113_layer_call_and_return_conditional_losses_1146328

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
 
_user_specified_nameinputs"?L
saver_filename:0StatefulPartitionedCall_1:0StatefulPartitionedCall_28"
saved_model_main_op

NoOp*>
__saved_model_init_op%#
__saved_model_init_op

NoOp*?
serving_default?
U
conv2d_112_inputA
"serving_default_conv2d_112_input:0?????????@@B
activation_2020
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
_tf_keras_sequential??{"class_name": "Sequential", "name": "sequential_28", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "must_restore_from_config": false, "config": {"name": "sequential_28", "layers": [{"class_name": "InputLayer", "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 64, 64, 3]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "conv2d_112_input"}}, {"class_name": "Conv2D", "config": {"name": "conv2d_112", "trainable": true, "batch_input_shape": {"class_name": "__tuple__", "items": [null, 64, 64, 3]}, "dtype": "float32", "filters": 60, "kernel_size": {"class_name": "__tuple__", "items": [5, 5]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_196", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_168", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "Conv2D", "config": {"name": "conv2d_113", "trainable": true, "dtype": "float32", "filters": 60, "kernel_size": {"class_name": "__tuple__", "items": [5, 5]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_197", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_169", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "MaxPooling2D", "config": {"name": "max_pooling2d_56", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}}, {"class_name": "Conv2D", "config": {"name": "conv2d_114", "trainable": true, "dtype": "float32", "filters": 30, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_198", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_170", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "Conv2D", "config": {"name": "conv2d_115", "trainable": true, "dtype": "float32", "filters": 30, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_199", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_171", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "MaxPooling2D", "config": {"name": "max_pooling2d_57", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}}, {"class_name": "Dropout", "config": {"name": "dropout_84", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}, {"class_name": "Flatten", "config": {"name": "flatten_56", "trainable": true, "dtype": "float32", "data_format": "channels_last"}}, {"class_name": "Dense", "config": {"name": "dense_84", "trainable": true, "dtype": "float32", "units": 300, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_200", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_172", "trainable": true, "dtype": "float32", "axis": [1], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "Dropout", "config": {"name": "dropout_85", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}, {"class_name": "Flatten", "config": {"name": "flatten_57", "trainable": true, "dtype": "float32", "data_format": "channels_last"}}, {"class_name": "Dense", "config": {"name": "dense_85", "trainable": true, "dtype": "float32", "units": 300, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_201", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_173", "trainable": true, "dtype": "float32", "axis": [1], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "Dropout", "config": {"name": "dropout_86", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}, {"class_name": "Dense", "config": {"name": "dense_86", "trainable": true, "dtype": "float32", "units": 6, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_202", "trainable": true, "dtype": "float32", "activation": "softmax"}}]}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 4, "axes": {"-1": 3}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 64, 64, 3]}, "is_graph_network": true, "keras_version": "2.4.0", "backend": "tensorflow", "model_config": {"class_name": "Sequential", "config": {"name": "sequential_28", "layers": [{"class_name": "InputLayer", "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 64, 64, 3]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "conv2d_112_input"}}, {"class_name": "Conv2D", "config": {"name": "conv2d_112", "trainable": true, "batch_input_shape": {"class_name": "__tuple__", "items": [null, 64, 64, 3]}, "dtype": "float32", "filters": 60, "kernel_size": {"class_name": "__tuple__", "items": [5, 5]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_196", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_168", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "Conv2D", "config": {"name": "conv2d_113", "trainable": true, "dtype": "float32", "filters": 60, "kernel_size": {"class_name": "__tuple__", "items": [5, 5]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_197", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_169", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "MaxPooling2D", "config": {"name": "max_pooling2d_56", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}}, {"class_name": "Conv2D", "config": {"name": "conv2d_114", "trainable": true, "dtype": "float32", "filters": 30, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_198", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_170", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "Conv2D", "config": {"name": "conv2d_115", "trainable": true, "dtype": "float32", "filters": 30, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_199", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_171", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "MaxPooling2D", "config": {"name": "max_pooling2d_57", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}}, {"class_name": "Dropout", "config": {"name": "dropout_84", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}, {"class_name": "Flatten", "config": {"name": "flatten_56", "trainable": true, "dtype": "float32", "data_format": "channels_last"}}, {"class_name": "Dense", "config": {"name": "dense_84", "trainable": true, "dtype": "float32", "units": 300, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_200", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_172", "trainable": true, "dtype": "float32", "axis": [1], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "Dropout", "config": {"name": "dropout_85", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}, {"class_name": "Flatten", "config": {"name": "flatten_57", "trainable": true, "dtype": "float32", "data_format": "channels_last"}}, {"class_name": "Dense", "config": {"name": "dense_85", "trainable": true, "dtype": "float32", "units": 300, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_201", "trainable": true, "dtype": "float32", "activation": "relu"}}, {"class_name": "BatchNormalization", "config": {"name": "batch_normalization_173", "trainable": true, "dtype": "float32", "axis": [1], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}}, {"class_name": "Dropout", "config": {"name": "dropout_86", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}, {"class_name": "Dense", "config": {"name": "dense_86", "trainable": true, "dtype": "float32", "units": 6, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "Activation", "config": {"name": "activation_202", "trainable": true, "dtype": "float32", "activation": "softmax"}}]}}, "training_config": {"loss": "categorical_crossentropy", "metrics": [[{"class_name": "MeanMetricWrapper", "config": {"name": "accuracy", "dtype": "float32", "fn": "categorical_accuracy"}}]], "weighted_metrics": null, "loss_weights": null, "optimizer_config": {"class_name": "Adam", "config": {"name": "Adam", "learning_rate": 0.0010000000474974513, "decay": 2.6666666599339806e-05, "beta_1": 0.8999999761581421, "beta_2": 0.9990000128746033, "epsilon": 1e-07, "amsgrad": false}}}}
?


"kernel
#bias
$	variables
%regularization_losses
&trainable_variables
'	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?	
_tf_keras_layer?	{"class_name": "Conv2D", "name": "conv2d_112", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": {"class_name": "__tuple__", "items": [null, 64, 64, 3]}, "stateful": false, "must_restore_from_config": false, "config": {"name": "conv2d_112", "trainable": true, "batch_input_shape": {"class_name": "__tuple__", "items": [null, 64, 64, 3]}, "dtype": "float32", "filters": 60, "kernel_size": {"class_name": "__tuple__", "items": [5, 5]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 4, "axes": {"-1": 3}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 64, 64, 3]}}
?
(	variables
)regularization_losses
*trainable_variables
+	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_196", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_196", "trainable": true, "dtype": "float32", "activation": "relu"}}
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
_tf_keras_layer?{"class_name": "BatchNormalization", "name": "batch_normalization_168", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "batch_normalization_168", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 4, "max_ndim": null, "min_ndim": null, "axes": {"3": 60}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 60, 60, 60]}}
?	

5kernel
6bias
7	variables
8regularization_losses
9trainable_variables
:	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Conv2D", "name": "conv2d_113", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "conv2d_113", "trainable": true, "dtype": "float32", "filters": 60, "kernel_size": {"class_name": "__tuple__", "items": [5, 5]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 4, "axes": {"-1": 60}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 60, 60, 60]}}
?
;	variables
<regularization_losses
=trainable_variables
>	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_197", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_197", "trainable": true, "dtype": "float32", "activation": "relu"}}
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
_tf_keras_layer?{"class_name": "BatchNormalization", "name": "batch_normalization_169", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "batch_normalization_169", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 4, "max_ndim": null, "min_ndim": null, "axes": {"3": 60}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 56, 56, 60]}}
?
H	variables
Iregularization_losses
Jtrainable_variables
K	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "MaxPooling2D", "name": "max_pooling2d_56", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "max_pooling2d_56", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 4, "max_ndim": null, "min_ndim": null, "axes": {}}}}
?	

Lkernel
Mbias
N	variables
Oregularization_losses
Ptrainable_variables
Q	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Conv2D", "name": "conv2d_114", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "conv2d_114", "trainable": true, "dtype": "float32", "filters": 30, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 4, "axes": {"-1": 60}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 28, 28, 60]}}
?
R	variables
Sregularization_losses
Ttrainable_variables
U	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_198", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_198", "trainable": true, "dtype": "float32", "activation": "relu"}}
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
_tf_keras_layer?{"class_name": "BatchNormalization", "name": "batch_normalization_170", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "batch_normalization_170", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 4, "max_ndim": null, "min_ndim": null, "axes": {"3": 30}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 26, 26, 30]}}
?	

_kernel
`bias
a	variables
bregularization_losses
ctrainable_variables
d	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Conv2D", "name": "conv2d_115", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "conv2d_115", "trainable": true, "dtype": "float32", "filters": 30, "kernel_size": {"class_name": "__tuple__", "items": [3, 3]}, "strides": {"class_name": "__tuple__", "items": [1, 1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1, 1]}, "groups": 1, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 4, "axes": {"-1": 30}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 26, 26, 30]}}
?
e	variables
fregularization_losses
gtrainable_variables
h	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_199", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_199", "trainable": true, "dtype": "float32", "activation": "relu"}}
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
_tf_keras_layer?{"class_name": "BatchNormalization", "name": "batch_normalization_171", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "batch_normalization_171", "trainable": true, "dtype": "float32", "axis": [3], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 4, "max_ndim": null, "min_ndim": null, "axes": {"3": 30}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 24, 24, 30]}}
?
r	variables
sregularization_losses
ttrainable_variables
u	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "MaxPooling2D", "name": "max_pooling2d_57", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "max_pooling2d_57", "trainable": true, "dtype": "float32", "pool_size": {"class_name": "__tuple__", "items": [2, 2]}, "padding": "valid", "strides": {"class_name": "__tuple__", "items": [2, 2]}, "data_format": "channels_last"}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 4, "max_ndim": null, "min_ndim": null, "axes": {}}}}
?
v	variables
wregularization_losses
xtrainable_variables
y	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Dropout", "name": "dropout_84", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "dropout_84", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}
?
z	variables
{regularization_losses
|trainable_variables
}	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Flatten", "name": "flatten_56", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "flatten_56", "trainable": true, "dtype": "float32", "data_format": "channels_last"}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 1, "axes": {}}}}
?

~kernel
bias
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Dense", "name": "dense_84", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "dense_84", "trainable": true, "dtype": "float32", "units": 300, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 4320}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 4320]}}
?
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_200", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_200", "trainable": true, "dtype": "float32", "activation": "relu"}}
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
_tf_keras_layer?{"class_name": "BatchNormalization", "name": "batch_normalization_172", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "batch_normalization_172", "trainable": true, "dtype": "float32", "axis": [1], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 2, "max_ndim": null, "min_ndim": null, "axes": {"1": 300}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 300]}}
?
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Dropout", "name": "dropout_85", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "dropout_85", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}
?
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Flatten", "name": "flatten_57", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "flatten_57", "trainable": true, "dtype": "float32", "data_format": "channels_last"}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 1, "axes": {}}}}
?
?kernel
	?bias
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Dense", "name": "dense_85", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "dense_85", "trainable": true, "dtype": "float32", "units": 300, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 300}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 300]}}
?
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_201", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_201", "trainable": true, "dtype": "float32", "activation": "relu"}}
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
_tf_keras_layer?{"class_name": "BatchNormalization", "name": "batch_normalization_173", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "batch_normalization_173", "trainable": true, "dtype": "float32", "axis": [1], "momentum": 0.99, "epsilon": 0.001, "center": true, "scale": true, "beta_initializer": {"class_name": "Zeros", "config": {}}, "gamma_initializer": {"class_name": "Ones", "config": {}}, "moving_mean_initializer": {"class_name": "Zeros", "config": {}}, "moving_variance_initializer": {"class_name": "Ones", "config": {}}, "beta_regularizer": null, "gamma_regularizer": null, "beta_constraint": null, "gamma_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 2, "max_ndim": null, "min_ndim": null, "axes": {"1": 300}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 300]}}
?
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Dropout", "name": "dropout_86", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "dropout_86", "trainable": true, "dtype": "float32", "rate": 0.4, "noise_shape": null, "seed": null}}
?
?kernel
	?bias
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Dense", "name": "dense_86", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "dense_86", "trainable": true, "dtype": "float32", "units": 6, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 300}}}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 300]}}
?
?	variables
?regularization_losses
?trainable_variables
?	keras_api
?__call__
+?&call_and_return_all_conditional_losses"?
_tf_keras_layer?{"class_name": "Activation", "name": "activation_202", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "config": {"name": "activation_202", "trainable": true, "dtype": "float32", "activation": "softmax"}}
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
+:)<2conv2d_112/kernel
:<2conv2d_112/bias
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
+:)<2batch_normalization_168/gamma
*:(<2batch_normalization_168/beta
3:1< (2#batch_normalization_168/moving_mean
7:5< (2'batch_normalization_168/moving_variance
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
+:)<<2conv2d_113/kernel
:<2conv2d_113/bias
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
+:)<2batch_normalization_169/gamma
*:(<2batch_normalization_169/beta
3:1< (2#batch_normalization_169/moving_mean
7:5< (2'batch_normalization_169/moving_variance
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
+:)<2conv2d_114/kernel
:2conv2d_114/bias
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
+:)2batch_normalization_170/gamma
*:(2batch_normalization_170/beta
3:1 (2#batch_normalization_170/moving_mean
7:5 (2'batch_normalization_170/moving_variance
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
+:)2conv2d_115/kernel
:2conv2d_115/bias
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
+:)2batch_normalization_171/gamma
*:(2batch_normalization_171/beta
3:1 (2#batch_normalization_171/moving_mean
7:5 (2'batch_normalization_171/moving_variance
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
?!?2dense_84/kernel
:?2dense_84/bias
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
,:*?2batch_normalization_172/gamma
+:)?2batch_normalization_172/beta
4:2? (2#batch_normalization_172/moving_mean
8:6? (2'batch_normalization_172/moving_variance
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
??2dense_85/kernel
:?2dense_85/bias
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
,:*?2batch_normalization_173/gamma
+:)?2batch_normalization_173/beta
4:2? (2#batch_normalization_173/moving_mean
8:6? (2'batch_normalization_173/moving_variance
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
": 	?2dense_86/kernel
:2dense_86/bias
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
0:.<2Adam/conv2d_112/kernel/m
": <2Adam/conv2d_112/bias/m
0:.<2$Adam/batch_normalization_168/gamma/m
/:-<2#Adam/batch_normalization_168/beta/m
0:.<<2Adam/conv2d_113/kernel/m
": <2Adam/conv2d_113/bias/m
0:.<2$Adam/batch_normalization_169/gamma/m
/:-<2#Adam/batch_normalization_169/beta/m
0:.<2Adam/conv2d_114/kernel/m
": 2Adam/conv2d_114/bias/m
0:.2$Adam/batch_normalization_170/gamma/m
/:-2#Adam/batch_normalization_170/beta/m
0:.2Adam/conv2d_115/kernel/m
": 2Adam/conv2d_115/bias/m
0:.2$Adam/batch_normalization_171/gamma/m
/:-2#Adam/batch_normalization_171/beta/m
(:&
?!?2Adam/dense_84/kernel/m
!:?2Adam/dense_84/bias/m
1:/?2$Adam/batch_normalization_172/gamma/m
0:.?2#Adam/batch_normalization_172/beta/m
(:&
??2Adam/dense_85/kernel/m
!:?2Adam/dense_85/bias/m
1:/?2$Adam/batch_normalization_173/gamma/m
0:.?2#Adam/batch_normalization_173/beta/m
':%	?2Adam/dense_86/kernel/m
 :2Adam/dense_86/bias/m
0:.<2Adam/conv2d_112/kernel/v
": <2Adam/conv2d_112/bias/v
0:.<2$Adam/batch_normalization_168/gamma/v
/:-<2#Adam/batch_normalization_168/beta/v
0:.<<2Adam/conv2d_113/kernel/v
": <2Adam/conv2d_113/bias/v
0:.<2$Adam/batch_normalization_169/gamma/v
/:-<2#Adam/batch_normalization_169/beta/v
0:.<2Adam/conv2d_114/kernel/v
": 2Adam/conv2d_114/bias/v
0:.2$Adam/batch_normalization_170/gamma/v
/:-2#Adam/batch_normalization_170/beta/v
0:.2Adam/conv2d_115/kernel/v
": 2Adam/conv2d_115/bias/v
0:.2$Adam/batch_normalization_171/gamma/v
/:-2#Adam/batch_normalization_171/beta/v
(:&
?!?2Adam/dense_84/kernel/v
!:?2Adam/dense_84/bias/v
1:/?2$Adam/batch_normalization_172/gamma/v
0:.?2#Adam/batch_normalization_172/beta/v
(:&
??2Adam/dense_85/kernel/v
!:?2Adam/dense_85/bias/v
1:/?2$Adam/batch_normalization_173/gamma/v
0:.?2#Adam/batch_normalization_173/beta/v
':%	?2Adam/dense_86/kernel/v
 :2Adam/dense_86/bias/v
?2?
/__inference_sequential_28_layer_call_fn_1145547
/__inference_sequential_28_layer_call_fn_1146080
/__inference_sequential_28_layer_call_fn_1146161
/__inference_sequential_28_layer_call_fn_1145359?
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
J__inference_sequential_28_layer_call_and_return_conditional_losses_1145849
J__inference_sequential_28_layer_call_and_return_conditional_losses_1145063
J__inference_sequential_28_layer_call_and_return_conditional_losses_1145999
J__inference_sequential_28_layer_call_and_return_conditional_losses_1145170?
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
"__inference__wrapped_model_1143583?
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
annotations? *7?4
2?/
conv2d_112_input?????????@@
?2?
,__inference_conv2d_112_layer_call_fn_1146180?
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
G__inference_conv2d_112_layer_call_and_return_conditional_losses_1146171?
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
0__inference_activation_196_layer_call_fn_1146190?
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
K__inference_activation_196_layer_call_and_return_conditional_losses_1146185?
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
9__inference_batch_normalization_168_layer_call_fn_1146318
9__inference_batch_normalization_168_layer_call_fn_1146241
9__inference_batch_normalization_168_layer_call_fn_1146305
9__inference_batch_normalization_168_layer_call_fn_1146254?
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
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_1146228
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_1146292
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_1146210
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_1146274?
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
,__inference_conv2d_113_layer_call_fn_1146337?
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
G__inference_conv2d_113_layer_call_and_return_conditional_losses_1146328?
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
0__inference_activation_197_layer_call_fn_1146347?
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
K__inference_activation_197_layer_call_and_return_conditional_losses_1146342?
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
9__inference_batch_normalization_169_layer_call_fn_1146475
9__inference_batch_normalization_169_layer_call_fn_1146462
9__inference_batch_normalization_169_layer_call_fn_1146398
9__inference_batch_normalization_169_layer_call_fn_1146411?
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
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_1146449
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_1146431
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_1146367
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_1146385?
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
2__inference_max_pooling2d_56_layer_call_fn_1143803?
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
M__inference_max_pooling2d_56_layer_call_and_return_conditional_losses_1143797?
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
,__inference_conv2d_114_layer_call_fn_1146494?
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
G__inference_conv2d_114_layer_call_and_return_conditional_losses_1146485?
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
0__inference_activation_198_layer_call_fn_1146504?
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
K__inference_activation_198_layer_call_and_return_conditional_losses_1146499?
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
9__inference_batch_normalization_170_layer_call_fn_1146619
9__inference_batch_normalization_170_layer_call_fn_1146568
9__inference_batch_normalization_170_layer_call_fn_1146555
9__inference_batch_normalization_170_layer_call_fn_1146632?
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
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_1146606
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_1146542
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_1146588
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_1146524?
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
,__inference_conv2d_115_layer_call_fn_1146651?
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
G__inference_conv2d_115_layer_call_and_return_conditional_losses_1146642?
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
0__inference_activation_199_layer_call_fn_1146661?
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
K__inference_activation_199_layer_call_and_return_conditional_losses_1146656?
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
9__inference_batch_normalization_171_layer_call_fn_1146776
9__inference_batch_normalization_171_layer_call_fn_1146725
9__inference_batch_normalization_171_layer_call_fn_1146712
9__inference_batch_normalization_171_layer_call_fn_1146789?
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
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_1146681
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_1146763
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_1146699
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_1146745?
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
2__inference_max_pooling2d_57_layer_call_fn_1144023?
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
M__inference_max_pooling2d_57_layer_call_and_return_conditional_losses_1144017?
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
,__inference_dropout_84_layer_call_fn_1146816
,__inference_dropout_84_layer_call_fn_1146811?
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
G__inference_dropout_84_layer_call_and_return_conditional_losses_1146801
G__inference_dropout_84_layer_call_and_return_conditional_losses_1146806?
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
,__inference_flatten_56_layer_call_fn_1146827?
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
G__inference_flatten_56_layer_call_and_return_conditional_losses_1146822?
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
*__inference_dense_84_layer_call_fn_1146846?
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
E__inference_dense_84_layer_call_and_return_conditional_losses_1146837?
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
0__inference_activation_200_layer_call_fn_1146856?
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
K__inference_activation_200_layer_call_and_return_conditional_losses_1146851?
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
9__inference_batch_normalization_172_layer_call_fn_1146925
9__inference_batch_normalization_172_layer_call_fn_1146938?
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
T__inference_batch_normalization_172_layer_call_and_return_conditional_losses_1146912
T__inference_batch_normalization_172_layer_call_and_return_conditional_losses_1146892?
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
,__inference_dropout_85_layer_call_fn_1146965
,__inference_dropout_85_layer_call_fn_1146960?
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
G__inference_dropout_85_layer_call_and_return_conditional_losses_1146950
G__inference_dropout_85_layer_call_and_return_conditional_losses_1146955?
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
,__inference_flatten_57_layer_call_fn_1146976?
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
G__inference_flatten_57_layer_call_and_return_conditional_losses_1146971?
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
*__inference_dense_85_layer_call_fn_1146995?
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
E__inference_dense_85_layer_call_and_return_conditional_losses_1146986?
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
0__inference_activation_201_layer_call_fn_1147005?
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
K__inference_activation_201_layer_call_and_return_conditional_losses_1147000?
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
9__inference_batch_normalization_173_layer_call_fn_1147087
9__inference_batch_normalization_173_layer_call_fn_1147074?
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
T__inference_batch_normalization_173_layer_call_and_return_conditional_losses_1147061
T__inference_batch_normalization_173_layer_call_and_return_conditional_losses_1147041?
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
,__inference_dropout_86_layer_call_fn_1147109
,__inference_dropout_86_layer_call_fn_1147114?
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
G__inference_dropout_86_layer_call_and_return_conditional_losses_1147099
G__inference_dropout_86_layer_call_and_return_conditional_losses_1147104?
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
*__inference_dense_86_layer_call_fn_1147133?
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
E__inference_dense_86_layer_call_and_return_conditional_losses_1147124?
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
0__inference_activation_202_layer_call_fn_1147143?
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
K__inference_activation_202_layer_call_and_return_conditional_losses_1147138?
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
%__inference_signature_wrapper_1145638conv2d_112_input"?
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
"__inference__wrapped_model_1143583?2"#-./056@ABCLMWXYZ_`jklm~????????????A?>
7?4
2?/
conv2d_112_input?????????@@
? "??<
:
activation_202(?%
activation_202??????????
K__inference_activation_196_layer_call_and_return_conditional_losses_1146185h7?4
-?*
(?%
inputs?????????<<<
? "-?*
#? 
0?????????<<<
? ?
0__inference_activation_196_layer_call_fn_1146190[7?4
-?*
(?%
inputs?????????<<<
? " ??????????<<<?
K__inference_activation_197_layer_call_and_return_conditional_losses_1146342h7?4
-?*
(?%
inputs?????????88<
? "-?*
#? 
0?????????88<
? ?
0__inference_activation_197_layer_call_fn_1146347[7?4
-?*
(?%
inputs?????????88<
? " ??????????88<?
K__inference_activation_198_layer_call_and_return_conditional_losses_1146499h7?4
-?*
(?%
inputs?????????
? "-?*
#? 
0?????????
? ?
0__inference_activation_198_layer_call_fn_1146504[7?4
-?*
(?%
inputs?????????
? " ???????????
K__inference_activation_199_layer_call_and_return_conditional_losses_1146656h7?4
-?*
(?%
inputs?????????
? "-?*
#? 
0?????????
? ?
0__inference_activation_199_layer_call_fn_1146661[7?4
-?*
(?%
inputs?????????
? " ???????????
K__inference_activation_200_layer_call_and_return_conditional_losses_1146851Z0?-
&?#
!?
inputs??????????
? "&?#
?
0??????????
? ?
0__inference_activation_200_layer_call_fn_1146856M0?-
&?#
!?
inputs??????????
? "????????????
K__inference_activation_201_layer_call_and_return_conditional_losses_1147000Z0?-
&?#
!?
inputs??????????
? "&?#
?
0??????????
? ?
0__inference_activation_201_layer_call_fn_1147005M0?-
&?#
!?
inputs??????????
? "????????????
K__inference_activation_202_layer_call_and_return_conditional_losses_1147138X/?,
%?"
 ?
inputs?????????
? "%?"
?
0?????????
? 
0__inference_activation_202_layer_call_fn_1147143K/?,
%?"
 ?
inputs?????????
? "???????????
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_1146210?-./0M?J
C?@
:?7
inputs+???????????????????????????<
p
? "??<
5?2
0+???????????????????????????<
? ?
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_1146228?-./0M?J
C?@
:?7
inputs+???????????????????????????<
p 
? "??<
5?2
0+???????????????????????????<
? ?
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_1146274r-./0;?8
1?.
(?%
inputs?????????<<<
p
? "-?*
#? 
0?????????<<<
? ?
T__inference_batch_normalization_168_layer_call_and_return_conditional_losses_1146292r-./0;?8
1?.
(?%
inputs?????????<<<
p 
? "-?*
#? 
0?????????<<<
? ?
9__inference_batch_normalization_168_layer_call_fn_1146241?-./0M?J
C?@
:?7
inputs+???????????????????????????<
p
? "2?/+???????????????????????????<?
9__inference_batch_normalization_168_layer_call_fn_1146254?-./0M?J
C?@
:?7
inputs+???????????????????????????<
p 
? "2?/+???????????????????????????<?
9__inference_batch_normalization_168_layer_call_fn_1146305e-./0;?8
1?.
(?%
inputs?????????<<<
p
? " ??????????<<<?
9__inference_batch_normalization_168_layer_call_fn_1146318e-./0;?8
1?.
(?%
inputs?????????<<<
p 
? " ??????????<<<?
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_1146367?@ABCM?J
C?@
:?7
inputs+???????????????????????????<
p
? "??<
5?2
0+???????????????????????????<
? ?
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_1146385?@ABCM?J
C?@
:?7
inputs+???????????????????????????<
p 
? "??<
5?2
0+???????????????????????????<
? ?
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_1146431r@ABC;?8
1?.
(?%
inputs?????????88<
p
? "-?*
#? 
0?????????88<
? ?
T__inference_batch_normalization_169_layer_call_and_return_conditional_losses_1146449r@ABC;?8
1?.
(?%
inputs?????????88<
p 
? "-?*
#? 
0?????????88<
? ?
9__inference_batch_normalization_169_layer_call_fn_1146398?@ABCM?J
C?@
:?7
inputs+???????????????????????????<
p
? "2?/+???????????????????????????<?
9__inference_batch_normalization_169_layer_call_fn_1146411?@ABCM?J
C?@
:?7
inputs+???????????????????????????<
p 
? "2?/+???????????????????????????<?
9__inference_batch_normalization_169_layer_call_fn_1146462e@ABC;?8
1?.
(?%
inputs?????????88<
p
? " ??????????88<?
9__inference_batch_normalization_169_layer_call_fn_1146475e@ABC;?8
1?.
(?%
inputs?????????88<
p 
? " ??????????88<?
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_1146524?WXYZM?J
C?@
:?7
inputs+???????????????????????????
p
? "??<
5?2
0+???????????????????????????
? ?
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_1146542?WXYZM?J
C?@
:?7
inputs+???????????????????????????
p 
? "??<
5?2
0+???????????????????????????
? ?
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_1146588rWXYZ;?8
1?.
(?%
inputs?????????
p
? "-?*
#? 
0?????????
? ?
T__inference_batch_normalization_170_layer_call_and_return_conditional_losses_1146606rWXYZ;?8
1?.
(?%
inputs?????????
p 
? "-?*
#? 
0?????????
? ?
9__inference_batch_normalization_170_layer_call_fn_1146555?WXYZM?J
C?@
:?7
inputs+???????????????????????????
p
? "2?/+????????????????????????????
9__inference_batch_normalization_170_layer_call_fn_1146568?WXYZM?J
C?@
:?7
inputs+???????????????????????????
p 
? "2?/+????????????????????????????
9__inference_batch_normalization_170_layer_call_fn_1146619eWXYZ;?8
1?.
(?%
inputs?????????
p
? " ???????????
9__inference_batch_normalization_170_layer_call_fn_1146632eWXYZ;?8
1?.
(?%
inputs?????????
p 
? " ???????????
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_1146681?jklmM?J
C?@
:?7
inputs+???????????????????????????
p
? "??<
5?2
0+???????????????????????????
? ?
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_1146699?jklmM?J
C?@
:?7
inputs+???????????????????????????
p 
? "??<
5?2
0+???????????????????????????
? ?
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_1146745rjklm;?8
1?.
(?%
inputs?????????
p
? "-?*
#? 
0?????????
? ?
T__inference_batch_normalization_171_layer_call_and_return_conditional_losses_1146763rjklm;?8
1?.
(?%
inputs?????????
p 
? "-?*
#? 
0?????????
? ?
9__inference_batch_normalization_171_layer_call_fn_1146712?jklmM?J
C?@
:?7
inputs+???????????????????????????
p
? "2?/+????????????????????????????
9__inference_batch_normalization_171_layer_call_fn_1146725?jklmM?J
C?@
:?7
inputs+???????????????????????????
p 
? "2?/+????????????????????????????
9__inference_batch_normalization_171_layer_call_fn_1146776ejklm;?8
1?.
(?%
inputs?????????
p
? " ???????????
9__inference_batch_normalization_171_layer_call_fn_1146789ejklm;?8
1?.
(?%
inputs?????????
p 
? " ???????????
T__inference_batch_normalization_172_layer_call_and_return_conditional_losses_1146892h????4?1
*?'
!?
inputs??????????
p
? "&?#
?
0??????????
? ?
T__inference_batch_normalization_172_layer_call_and_return_conditional_losses_1146912h????4?1
*?'
!?
inputs??????????
p 
? "&?#
?
0??????????
? ?
9__inference_batch_normalization_172_layer_call_fn_1146925[????4?1
*?'
!?
inputs??????????
p
? "????????????
9__inference_batch_normalization_172_layer_call_fn_1146938[????4?1
*?'
!?
inputs??????????
p 
? "????????????
T__inference_batch_normalization_173_layer_call_and_return_conditional_losses_1147041h????4?1
*?'
!?
inputs??????????
p
? "&?#
?
0??????????
? ?
T__inference_batch_normalization_173_layer_call_and_return_conditional_losses_1147061h????4?1
*?'
!?
inputs??????????
p 
? "&?#
?
0??????????
? ?
9__inference_batch_normalization_173_layer_call_fn_1147074[????4?1
*?'
!?
inputs??????????
p
? "????????????
9__inference_batch_normalization_173_layer_call_fn_1147087[????4?1
*?'
!?
inputs??????????
p 
? "????????????
G__inference_conv2d_112_layer_call_and_return_conditional_losses_1146171l"#7?4
-?*
(?%
inputs?????????@@
? "-?*
#? 
0?????????<<<
? ?
,__inference_conv2d_112_layer_call_fn_1146180_"#7?4
-?*
(?%
inputs?????????@@
? " ??????????<<<?
G__inference_conv2d_113_layer_call_and_return_conditional_losses_1146328l567?4
-?*
(?%
inputs?????????<<<
? "-?*
#? 
0?????????88<
? ?
,__inference_conv2d_113_layer_call_fn_1146337_567?4
-?*
(?%
inputs?????????<<<
? " ??????????88<?
G__inference_conv2d_114_layer_call_and_return_conditional_losses_1146485lLM7?4
-?*
(?%
inputs?????????<
? "-?*
#? 
0?????????
? ?
,__inference_conv2d_114_layer_call_fn_1146494_LM7?4
-?*
(?%
inputs?????????<
? " ???????????
G__inference_conv2d_115_layer_call_and_return_conditional_losses_1146642l_`7?4
-?*
(?%
inputs?????????
? "-?*
#? 
0?????????
? ?
,__inference_conv2d_115_layer_call_fn_1146651__`7?4
-?*
(?%
inputs?????????
? " ???????????
E__inference_dense_84_layer_call_and_return_conditional_losses_1146837^~0?-
&?#
!?
inputs??????????!
? "&?#
?
0??????????
? 
*__inference_dense_84_layer_call_fn_1146846Q~0?-
&?#
!?
inputs??????????!
? "????????????
E__inference_dense_85_layer_call_and_return_conditional_losses_1146986`??0?-
&?#
!?
inputs??????????
? "&?#
?
0??????????
? ?
*__inference_dense_85_layer_call_fn_1146995S??0?-
&?#
!?
inputs??????????
? "????????????
E__inference_dense_86_layer_call_and_return_conditional_losses_1147124_??0?-
&?#
!?
inputs??????????
? "%?"
?
0?????????
? ?
*__inference_dense_86_layer_call_fn_1147133R??0?-
&?#
!?
inputs??????????
? "???????????
G__inference_dropout_84_layer_call_and_return_conditional_losses_1146801l;?8
1?.
(?%
inputs?????????
p
? "-?*
#? 
0?????????
? ?
G__inference_dropout_84_layer_call_and_return_conditional_losses_1146806l;?8
1?.
(?%
inputs?????????
p 
? "-?*
#? 
0?????????
? ?
,__inference_dropout_84_layer_call_fn_1146811_;?8
1?.
(?%
inputs?????????
p
? " ???????????
,__inference_dropout_84_layer_call_fn_1146816_;?8
1?.
(?%
inputs?????????
p 
? " ???????????
G__inference_dropout_85_layer_call_and_return_conditional_losses_1146950^4?1
*?'
!?
inputs??????????
p
? "&?#
?
0??????????
? ?
G__inference_dropout_85_layer_call_and_return_conditional_losses_1146955^4?1
*?'
!?
inputs??????????
p 
? "&?#
?
0??????????
? ?
,__inference_dropout_85_layer_call_fn_1146960Q4?1
*?'
!?
inputs??????????
p
? "????????????
,__inference_dropout_85_layer_call_fn_1146965Q4?1
*?'
!?
inputs??????????
p 
? "????????????
G__inference_dropout_86_layer_call_and_return_conditional_losses_1147099^4?1
*?'
!?
inputs??????????
p
? "&?#
?
0??????????
? ?
G__inference_dropout_86_layer_call_and_return_conditional_losses_1147104^4?1
*?'
!?
inputs??????????
p 
? "&?#
?
0??????????
? ?
,__inference_dropout_86_layer_call_fn_1147109Q4?1
*?'
!?
inputs??????????
p
? "????????????
,__inference_dropout_86_layer_call_fn_1147114Q4?1
*?'
!?
inputs??????????
p 
? "????????????
G__inference_flatten_56_layer_call_and_return_conditional_losses_1146822a7?4
-?*
(?%
inputs?????????
? "&?#
?
0??????????!
? ?
,__inference_flatten_56_layer_call_fn_1146827T7?4
-?*
(?%
inputs?????????
? "???????????!?
G__inference_flatten_57_layer_call_and_return_conditional_losses_1146971Z0?-
&?#
!?
inputs??????????
? "&?#
?
0??????????
? }
,__inference_flatten_57_layer_call_fn_1146976M0?-
&?#
!?
inputs??????????
? "????????????
M__inference_max_pooling2d_56_layer_call_and_return_conditional_losses_1143797?R?O
H?E
C?@
inputs4????????????????????????????????????
? "H?E
>?;
04????????????????????????????????????
? ?
2__inference_max_pooling2d_56_layer_call_fn_1143803?R?O
H?E
C?@
inputs4????????????????????????????????????
? ";?84?????????????????????????????????????
M__inference_max_pooling2d_57_layer_call_and_return_conditional_losses_1144017?R?O
H?E
C?@
inputs4????????????????????????????????????
? "H?E
>?;
04????????????????????????????????????
? ?
2__inference_max_pooling2d_57_layer_call_fn_1144023?R?O
H?E
C?@
inputs4????????????????????????????????????
? ";?84?????????????????????????????????????
J__inference_sequential_28_layer_call_and_return_conditional_losses_1145063?2"#-./056@ABCLMWXYZ_`jklm~????????????I?F
??<
2?/
conv2d_112_input?????????@@
p

 
? "%?"
?
0?????????
? ?
J__inference_sequential_28_layer_call_and_return_conditional_losses_1145170?2"#-./056@ABCLMWXYZ_`jklm~????????????I?F
??<
2?/
conv2d_112_input?????????@@
p 

 
? "%?"
?
0?????????
? ?
J__inference_sequential_28_layer_call_and_return_conditional_losses_1145849?2"#-./056@ABCLMWXYZ_`jklm~??????????????<
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
J__inference_sequential_28_layer_call_and_return_conditional_losses_1145999?2"#-./056@ABCLMWXYZ_`jklm~??????????????<
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
/__inference_sequential_28_layer_call_fn_1145359?2"#-./056@ABCLMWXYZ_`jklm~????????????I?F
??<
2?/
conv2d_112_input?????????@@
p

 
? "???????????
/__inference_sequential_28_layer_call_fn_1145547?2"#-./056@ABCLMWXYZ_`jklm~????????????I?F
??<
2?/
conv2d_112_input?????????@@
p 

 
? "???????????
/__inference_sequential_28_layer_call_fn_1146080?2"#-./056@ABCLMWXYZ_`jklm~??????????????<
5?2
(?%
inputs?????????@@
p

 
? "???????????
/__inference_sequential_28_layer_call_fn_1146161?2"#-./056@ABCLMWXYZ_`jklm~??????????????<
5?2
(?%
inputs?????????@@
p 

 
? "???????????
%__inference_signature_wrapper_1145638?2"#-./056@ABCLMWXYZ_`jklm~????????????U?R
? 
K?H
F
conv2d_112_input2?/
conv2d_112_input?????????@@"??<
:
activation_202(?%
activation_202?????????