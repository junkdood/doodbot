#!/usr/bin/python3
# coding:utf-8 
import os
import tensorflow as tf
from tensorflow.keras import layers, models
import gzip
import numpy as np
import cv2

config = {
    'check_path': "./src/doodbot/CNNdata/modelckpt/cp-{epoch:04d}.ckpt",
    'class_num': 27,
    'data_sets': [
        './src/doodbot/CNNdata/dataset/emnist-letters-train-labels-idx1-ubyte.gz',
        './src/doodbot/CNNdata/dataset/emnist-letters-train-images-idx3-ubyte.gz',
        './src/doodbot/CNNdata/dataset/emnist-letters-test-labels-idx1-ubyte.gz',
        './src/doodbot/CNNdata/dataset/emnist-letters-test-images-idx3-ubyte.gz']
}

config2 = {
    'check_path': "./src/doodbot/CNNdata/modelckpt2/cp-{epoch:04d}.ckpt",
    'class_num': 3,
    'data_sets': [
        './src/doodbot/CNNdata/dataset/emnist-letters-train-labels-idx1-ubyte.gz',
        './src/doodbot/CNNdata/dataset/emnist-letters-train-images-idx3-ubyte.gz',
        './src/doodbot/CNNdata/dataset/emnist-letters-test-labels-idx1-ubyte.gz',
        './src/doodbot/CNNdata/dataset/emnist-letters-test-images-idx3-ubyte.gz']
}



def read_idx3(filename):
    with gzip.open(filename) as fo:
        print('Reading images...')
        buf = fo.read()
        offset = 0 
        header = np.frombuffer(buf, dtype='>i', count=4, offset=offset)
        print(header)
        magic_number, num_images, num_rows, num_cols = header
        print("\tmagic number: {}, number of images: {}, number of rows: {}, number of columns: {}".format(magic_number, num_images, num_rows, num_cols))
        offset += header.size * header.itemsize
        data = np.frombuffer(buf, '>B', num_images * num_rows * num_cols, offset).reshape(
            (num_images, num_rows, num_cols))
        return data, num_images


def read_idx1(filename):
    with gzip.open(filename) as fo:
        print('Reading labels...')
        buf = fo.read()
        offset = 0
        header = np.frombuffer(buf, '>i', 2, offset)
        magic_number, num_labels = header
        print("\tmagic number: {}, number of labels: {}".format(magic_number, num_labels))
        offset += header.size * header.itemsize
        data = np.frombuffer(buf, '>B', num_labels, offset)
        return data


class CNN(object):
    def __init__(self):
        self.model = self.model()

    @staticmethod
    def model():
        # LetNet
        model = models.Sequential()
        model.add(layers.Conv2D(32, (3, 3), activation='relu', input_shape=(28, 28, 1)))
        model.add(layers.MaxPooling2D((2, 2)))
        model.add(layers.Conv2D(64, (3, 3), activation='relu'))
        model.add(layers.MaxPooling2D((2, 2)))
        model.add(layers.Conv2D(64, (3, 3), activation='relu'))
        model.add(layers.Flatten())

        
        model.add(layers.Dense(64, activation='relu'))
        model.add(layers.Dense(config['class_num'], activation=tf.nn.softmax, name='predictions'))
        # model.summary()
        return model

class getData(object):
    def __init__(self):
        files = config['data_sets']

        train_labels = read_idx1(files[0])
        train_images, train_images_num = read_idx3(files[1])
        test_labels = read_idx1(files[2])
        test_images, test_images_num = read_idx3(files[3])

        train_images = train_images.reshape((train_images_num, 28, 28, 1))
        test_images = test_images.reshape((test_images_num, 28, 28, 1))

        train_images, test_images = train_images / 255.0, test_images / 255.0

        self.train_images, self.train_labels = train_images, train_labels
        self.test_images, self.test_labels = test_images, test_labels


class Train:
    def __init__(self):
        self.cnn = CNN()
        self.data = getData()

    def train(self):
        check_path = config['check_path']
        save_model_cb = tf.keras.callbacks.ModelCheckpoint(check_path, save_weights_only=True, verbose=1, period=5)
        self.cnn.model.compile(optimizer='adam',
                               loss='sparse_categorical_crossentropy',
                               metrics=['accuracy'])


        self.cnn.model.fit(self.data.train_images, self.data.train_labels, epochs=20, callbacks=[save_model_cb])

        test_loss, test_acc = self.cnn.model.evaluate(self.data.test_images, self.data.test_labels)
        print("准确率: %.4f，共测试了%d张图片 " % (test_acc, len(self.data.test_labels)))


class CNN2(object):
    def __init__(self):
        self.model = self.model()

    @staticmethod
    def model():
        # LetNet
        model = models.Sequential()
        model.add(layers.Conv2D(32, (3, 3), activation='relu', input_shape=(28, 28, 1)))
        model.add(layers.MaxPooling2D((2, 2)))
        model.add(layers.Conv2D(64, (3, 3), activation='relu'))
        model.add(layers.MaxPooling2D((2, 2)))
        model.add(layers.Conv2D(64, (3, 3), activation='relu'))
        model.add(layers.Flatten())

        
        model.add(layers.Dense(64, activation='relu'))
        model.add(layers.Dense(config2['class_num'], activation=tf.nn.softmax, name='predictions'))
        # model.summary()
        return model

class getData2(object):
    def __init__(self):
        files = config2['data_sets']

        train_labels = read_idx1(files[0])
        train_images, train_images_num = read_idx3(files[1])
        test_labels = read_idx1(files[2])
        test_images, test_images_num = read_idx3(files[3])

        train_images = train_images.reshape((train_images_num, 28, 28, 1))
        test_images = test_images.reshape((test_images_num, 28, 28, 1))

        train_images, test_images = train_images / 255.0, test_images / 255.0

        train_filter = np.where((train_labels == 15 ) | (train_labels == 24))
        test_filter = np.where((test_labels == 15) | (test_labels == 24))

        train_images, train_labels = train_images[train_filter], train_labels[train_filter]
        test_images, test_labels = test_images[test_filter], test_labels[test_filter]

        O = 0
        X = 0
        for i in range(len(train_labels)):
            if train_labels[i] == 15:
                train_labels[i] = 1
                O+=1
            elif train_labels[i] == 24:
                train_labels[i] = 2
                X+=1
            else:
                print("wrong")

        print("trainO:",O)
        print("trainX:",X)

        O = 0
        X = 0
        for i in range(len(test_labels)):
            if test_labels[i] == 15:
                test_labels[i] = 1
                O+=1
            elif test_labels[i] == 24:
                test_labels[i] = 2
                X+=1
            else:
                print("wrong")

        print("testO:",O)
        print("testX:",X)

        
        
        train_labels = np.append(train_labels,np.zeros(4800))
        train_images = np.concatenate((train_images,np.array([0]*4800*28*28*1).reshape(4800, 28,28,1)),axis=0)

        test_labels = np.append(test_labels,np.zeros(4800))
        test_images = np.concatenate((test_images,np.array([0]*4800*28*28*1).reshape(4800, 28,28,1)),axis=0)

        state = np.random.get_state()
        np.random.set_state(state)
        np.random.shuffle(train_labels)
        np.random.set_state(state)
        np.random.shuffle(train_images)


        state = np.random.get_state()
        np.random.set_state(state)
        np.random.shuffle(test_labels)
        np.random.set_state(state)
        np.random.shuffle(test_images)


        self.train_images, self.train_labels = train_images, train_labels
        self.test_images, self.test_labels = test_images, test_labels


class Train2:
    def __init__(self):
        self.cnn = CNN2()
        self.data = getData2()

    def train(self):
        check_path = config2['check_path']
        save_model_cb = tf.keras.callbacks.ModelCheckpoint(check_path, save_weights_only=True, verbose=1, period=5)
        self.cnn.model.compile(optimizer='adam',
                               loss='sparse_categorical_crossentropy',
                               metrics=['accuracy'])


        self.cnn.model.fit(self.data.train_images, self.data.train_labels, epochs=20, callbacks=[save_model_cb])

        test_loss, test_acc = self.cnn.model.evaluate(self.data.test_images, self.data.test_labels)
        print("准确率: %.4f，共测试了%d张图片 " % (test_acc, len(self.data.test_labels)))

if __name__ == "__main__":
    trainer = Train2()
    trainer.train()