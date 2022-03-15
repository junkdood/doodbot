#!/usr/bin/python3
# coding:utf-8 
import os
import tensorflow as tf
from tensorflow.keras import layers, models
import gzip
import numpy as np

config = {
    'check_path': "./src/doodbot/CNNdata/modelckpt/cp-{epoch:04d}.ckpt",
    'class_num': 27,
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

if __name__ == "__main__":
    trainer = Train()
    trainer.train()