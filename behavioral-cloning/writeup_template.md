# **Behavioral Cloning**

## Writeup Template

---

**Behavioral Cloning Project**

#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network
* writeup_report.md or writeup_report.pdf summarizing the results

#### 2. Submission includes functional code

<b>!! Download the pre-trained model:</b> https://jumpshare.com/v/7lnxpaJDY0IU9BVAyZCv

To test the model, you first need to launch the python script drive.py. This one will create a socket server used to send steering angle predictions.

Using the Udacity provided simulator and the drive.py file, the car can be driven autonomously around the track by executing:

```sh
python drive.py model.h5
```

Then you can launch the Udacity provided simulator.

#### 3. Submission code is usable and readable

The model.py is composed of 4 functions:
    <ul>
        <li><b>build_model: Method used to build the architecture with keras.</b></li>
        <li><b>plot_image: Utils method used to plot an image extracted from the dataset.</b></li>
        <li><b>get_data: Generator method used to create batches used train the model.</b></li>
        <li><b>main: Function used to open the dataset, split it (training, validation) and train the model.</b></li>
    </ul>

Train the model:
```sh
python model.py
```

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed

I train the model using a convolutional neural network predicting one linear output. As I said above, the architecture is built in the build_method (model.py) method.

#### 2. Attempts to reduce overfitting in the model

In order to reduce the overfitting, I used data augmentation by flipping each image of the dataset and by using the left/right camera. I also add dropout just after the last convolution and after the first Fully connected layer.

#### 3. Model parameter tuning

The model used an Adam optimizer, so the learning rate was not tuned manually. Then, for the architecture, the size and the number of filters have been set by many trial and error.

#### 4. Appropriate training data

For the training data, I used a combination of the central, left and right camera, each one, randomly flip. The training data is a subset of 85% of the total dataset.

### Model Architecture and Training Strategy

#### 1. Solution Design Approach

The overall strategy for deriving a model architecture was by trial and errors. I first try to use the model provided by NVIDIA, but I stayed stuck to bad results. This was certainly due to some bad implementations in the processing of the dataset. After refactoring my code and modifier the architecture by another one, I've started to get some better outcoms.

I think, the problem was not really located into the NVIDIA model, but more on the processing of the dataset on my side. This is the reason why I do not use the model from NVIDIA but another.

Since it was the first time I used Keras I had trouble training this model. I think the main error I did, was to not be enough patient before to test the model on the simulator. I thought the architecture was bad whereas it just needed more time to be trained properly.

#### 2. Final Model Architecture

The architecture is the following:
<ul>
    <li>One convolution of 8 filters (9*9) [Elu activation]</li>
    <li>One convolution of 16 filters (5*5) [Elu activation]</li>
    <li>One convolution of 32 filters (4*4) [Elu activation]</li>
    <li>Flatten layer</li>
    <li>Dropout: 0.6</li>
    <li>Fully conected: 1024 [Elu]</li>
    <li>Dropout: 0.3</li>
    <li>Fully conected: 1 (Linear output)</li>
</ul>

#### 3. Creation of the Training Set & Training Process

In the get_data method, I start to shuffle the dataset. Then, I go through each row where I randomly pick the center, left or right image. Then, I randomly choose to flip the image or not. If the image is flipped I multiply the angle by -1. Each time a new batch is ready, I return it using yield (Generator).

<img src="images/left.png" /><img src="images/right.png" /><img src="images/center.png" />

Before to be fitted into the network, each image is normalized by the following formula: (images / 127.5 - 1). Then I cropped it to remove useless image part (Sky, tree... etc).
