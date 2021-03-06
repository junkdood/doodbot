#!/usr/bin/python3
# coding:utf-8 
import os
import rospy
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
    'BP_path': "./src/doodbot/CNNdata/modelBP/result.npz",
    'class_num': 4,
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
        print("?????????: %.4f???????????????%d????????? " % (test_acc, len(self.data.test_labels)))


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

        
        # ????????????
        train_labels = np.append(train_labels,np.zeros(4800))
        train_images = np.concatenate((train_images,np.array([0]*4800*28*28*1).reshape(4800, 28,28,1)),axis=0)
        test_labels = np.append(test_labels,np.zeros(800))
        test_images = np.concatenate((test_images,np.array([0]*800*28*28*1).reshape(800, 28,28,1)),axis=0)

        # ????????????
        train_labels = np.append(train_labels,np.zeros(4800)+3)
        train_images = np.concatenate((train_images,np.array([1]*4800*28*28*1).reshape(4800, 28,28,1)),axis=0)
        test_labels = np.append(test_labels,np.zeros(800)+3)
        test_images = np.concatenate((test_images,np.array([1]*800*28*28*1).reshape(800, 28,28,1)),axis=0)

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

def active_f(x):
    return np.maximum(0, x)
def active_f_g(x):
    return (x >= 0).astype(int)

class BP:
    def __init__(self):
        #???????????????
        input_nodes = 784
        hidden_nodes = 50
        output_nodes = config2['class_num']
        learning_rate = 0.01
        self.__weight1 = np.random.normal(0.0, pow(hidden_nodes, -0.5), (hidden_nodes, input_nodes))
        self.__weight2 = np.random.normal(0.0, pow(output_nodes, -0.5), (output_nodes, hidden_nodes))
        self.__learning_rate = learning_rate

    def train(self, t_input, t_target):
        #????????????
        inputs = np.array(t_input, ndmin=2).T
        targets = np.array(t_target, ndmin=2).T
        hidden_inputs = np.dot(self.__weight1, inputs)
        hidden_outputs = active_f(hidden_inputs)
        final_inputs = np.dot(self.__weight2, hidden_outputs)
        final_outputs = active_f(final_inputs)
        
        #???????????????????????????
        output_errors = targets - final_outputs
        hidden_errors = np.dot(self.__weight2.T, output_errors)
        self.__weight2 += self.__learning_rate * np.dot((output_errors * active_f_g(final_inputs)),np.transpose(hidden_outputs))
        self.__weight1 += self.__learning_rate * np.dot((hidden_errors * active_f_g(hidden_inputs)),(np.transpose(inputs)))
 
    def predict(self, inputs_list):
        #???????????????????????????????????????????????????
        inputs = np.array(inputs_list, ndmin=2).T
        hidden_inputs = np.dot(self.__weight1, inputs)
        hidden_outputs = active_f(hidden_inputs)
        final_inputs = np.dot(self.__weight2, hidden_outputs)
        final_outputs = active_f(final_inputs)
        return final_outputs

    def saveweight(self, path):
        np.savez(path, weight1 = self.__weight1, weight2 = self.__weight2)

    def loadweight(self, path):
        weight = np.load(path)
        self.__weight1 = weight['weight1']
        self.__weight2 = weight['weight2']



class Train2:
    def __init__(self):
        self.cnn = CNN2()
        self.data = getData2()

        self.bp = BP()

    def train(self):
        check_path = config2['check_path']
        save_model_cb = tf.keras.callbacks.ModelCheckpoint(check_path, save_weights_only=True, verbose=1, period=5)
        self.cnn.model.compile(optimizer='adam',
                               loss='sparse_categorical_crossentropy',
                               metrics=['accuracy'])


        self.cnn.model.fit(self.data.train_images, self.data.train_labels, epochs=20, callbacks=[save_model_cb])

        test_loss, test_acc = self.cnn.model.evaluate(self.data.test_images, self.data.test_labels)
        print("?????????: %.4f???????????????%d????????? " % (test_acc, len(self.data.test_labels)))
        begin_t = rospy.Time.now()
        Num = self.cnn.model.predict(np.array([self.data.test_images[0]]))
        end_t = rospy.Time.now()
        print("Duration: {}".format((end_t - begin_t).to_sec()))


    def trainBP(self):
        for epo in range(3):
            print(epo)
            for i in range(len(self.data.train_images)):
                #??????train????????????
                label = [0,0,0,0]
                label[int(self.data.train_labels[i])] = 1
                self.bp.train(self.data.train_images[i].reshape(784),label)
        self.bp.saveweight(config2['BP_path'])
        acc = []
        for i in range(len(self.data.test_images)):
            label = np.argmax(self.bp.predict(self.data.test_images[i].reshape(784)))
            #????????????????????????????????????????????????????????????????????????
            if int(self.data.test_labels[i]) == label:
                acc.append(1)
            else:
                acc.append(0)
        #?????????????????????
        print("acc is ", np.array(acc).mean())
        begin_t = rospy.Time.now()
        Num = self.bp.predict(np.array([1]*28*28*1).reshape(784))
        print(Num)
        end_t = rospy.Time.now()
        print("Duration: {}".format((end_t - begin_t).to_sec()))



if __name__ == "__main__":
    rospy.init_node('train')
    trainer = Train2()
    trainer.train()
    # trainer.trainBP()