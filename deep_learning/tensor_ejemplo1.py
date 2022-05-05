# File        :   main.py (FFNN for Digit Recognition)
# Version     :   1.1.0
# Description :   Script that implements a non-ConvNet FFNN for
#             :   digit recognition using keras and MNIST
# Date:       :   Feb 01, 2022
# Author      :   Ricardo Acevedo-Avila (racevedoaa@gmail.com)
# License     :   MIT

import tensorflow as tf
import matplotlib.pyplot as plt
import numpy as np
import cv2

# seed the pseudorandom number generator
from random import seed
from random import randint


# Shows an image
def showImage(imageName, inputImage, delay=0):
    cv2.namedWindow(imageName, cv2.WINDOW_NORMAL)
    cv2.imshow(imageName, inputImage)
    cv2.waitKey(delay)


# Writes a png image to disk:
def writeImage(imagePath, inputImage):
    imagePath = imagePath + ".png"
    cv2.imwrite(imagePath, inputImage, [cv2.IMWRITE_PNG_COMPRESSION, 0])
    print("Wrote Image: " + imagePath)


outPath = "/home/elio987/Documents/puzzlebot-a.k.a-KIRB/deep_learning/CNN/numberClassifier/"

print("Tensorflow version: " + str(tf.__version__))

# Total number classes:
classes = 10  # Numbers 0-9
sampleSize = (28, 28)  # size of the images

# Hyper parameters:
batchSize = 32 # Size of the batch (32/64)
epochs = 5     # Number of epochs

# Import the MNIST dataset:
# Images of 28 x 28 hand written digits 0 - 9:
mnist = tf.keras.datasets.mnist

# Unpack the dataset:
# "x" is the vector of images
# "y" is the vector of labels:
(x_train, y_train), (x_test, y_test) = mnist.load_data()

# Check out the dimensions:
print("Train Samples: "+str(x_train.shape[0]))
print("Test Samples: "+str(x_test.shape[0]))

# Show a sample with black/white color map:
plt.imshow(x_train[0], cmap=plt.cm.binary)
plt.show()
# Show image via OpenCV:)
showImage("Input Sample", x_train[0])

# Normalize/Scale the dataset:
x_train = x_train.astype(np.float) / 255.0
x_test = x_test.astype(np.float) / 255.0

# Show a scaled sample with black/white color  map:
plt.imshow(x_train[0], cmap=plt.cm.binary)
plt.show()

# Build the model, it is sequential model (a FNN)
model = tf.keras.Sequential()

# Build the Network layer by layer:

# Input Layer:
model.add(tf.keras.layers.Flatten(input_shape=sampleSize))

# Hidden layers:
# Add a layer with 128 neurons:
model.add(tf.keras.layers.Dense(128))
# Add the activavion layer:
model.add(tf.keras.layers.Activation("relu"))

# Add the output layer:
# The output is one of 10 possible classes:
model.add(tf.keras.layers.Dense(classes))
# Add the activation layer:
model.add(tf.keras.layers.Activation("softmax"))

# Train the model:
model.compile(loss="sparse_categorical_crossentropy", optimizer="adam", metrics=["accuracy"])

# Verbose fitting:
evalData = model.fit(
    x_train,
    y_train,
    batch_size=batchSize,
    epochs=epochs,
    verbose=1,
    # We pass some validation for
    # monitoring validation loss and metrics
    # at the end of each epoch
    validation_data=(x_test, y_test),
)

# Asses the model:
val_loss, val_acc = model.evaluate(x_test, y_test)
print((val_loss, val_acc))

plt.figure()
N = np.arange(0, epochs)
plt.plot(N, evalData.history["loss"], label="train_loss")
plt.plot(N, evalData.history["val_loss"], label="val_loss")
plt.plot(N, evalData.history["acc"], label="train_acc")
plt.plot(N, evalData.history["val_acc"], label="val_acc")
plt.title("Training Loss and Accuracy on Dataset")
plt.xlabel("Epoch #")
plt.ylabel("Loss/Accuracy")
plt.legend(loc="lower left")
plt.show()

# Save the model:
modelFilename = "numberRecognizer.model"
model.save(outPath + modelFilename)

# Load model:
numberRecognizer = tf.keras.models.load_model(outPath + modelFilename)

# Classify samples:
predictions = numberRecognizer.predict(x_test)

# Print the predictions:
print(predictions)

# Review some classifications:
testClasses = 15  # len(predictions)

# Seed the RNG:
seed(15)

for i in range(testClasses):
    # Get random number:
    randomNumber = randint(0, 10)
    # Get the largest prediction:
    largestPrediction = np.argmax(predictions[randomNumber])

    # Print it:
    print("FNN says: " + str(largestPrediction))

    # Show the classified sample:
    plt.title("Number: "+str(largestPrediction))
    plt.imshow(x_test[randomNumber], cmap=plt.cm.binary)
    plt.show()
