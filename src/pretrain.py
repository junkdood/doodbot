#!/usr/bin/python3
# coding:utf-8 
import os
import rospy
import torch
import torch.nn as nn
import gzip
import numpy as np
import cv2

cudaIdx = "cuda:0"  # GPU card index
device = torch.device(cudaIdx if torch.cuda.is_available() else "cpu")

config = {
    'check_path': "./src/doodbot/CNNdata/modelpth/model.pth",
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

class getData(object):
    def __init__(self):
        files = config['data_sets']

        train_labels = read_idx1(files[0])
        train_images, train_images_num = read_idx3(files[1])
        test_labels = read_idx1(files[2])
        test_images, test_images_num = read_idx3(files[3])

        train_images = train_images.reshape((train_images_num, 1, 28, 28))
        test_images = test_images.reshape((test_images_num, 1, 28, 28))

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

        
        # 全白样本
        train_labels = np.append(train_labels,np.zeros(4800))
        train_images = np.concatenate((train_images,np.array([0]*4800*28*28*1).reshape(4800,1, 28,28)),axis=0)
        test_labels = np.append(test_labels,np.zeros(800))
        test_images = np.concatenate((test_images,np.array([0]*800*28*28*1).reshape(800,1, 28,28)),axis=0)

        # 全黑样本
        train_labels = np.append(train_labels,np.zeros(4800)+3)
        train_images = np.concatenate((train_images,np.array([1]*4800*28*28*1).reshape(4800,1, 28,28)),axis=0)
        test_labels = np.append(test_labels,np.zeros(800)+3)
        test_images = np.concatenate((test_images,np.array([1]*800*28*28*1).reshape(800,1, 28,28)),axis=0)

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


# 定义神经网络
class CNN(nn.Module):
    def __init__(self):
        super(CNN, self).__init__()
        self.conv1 = nn.Sequential(  
            # input shape (1, 28, 28)
            nn.Conv2d(
                in_channels=1,  # 输入通道数
                out_channels=32,  # 输出通道数
                kernel_size=3,   # 卷积核大小          
                stride=1,  #卷积步数
                padding=0  # 如果想要 con2d 出来的图片长宽没有变化,padding=(kernel_size-1)/2 当 stride=1
            ),
            # output shape (32, 26, 26)
            nn.ReLU(),  # activation
            nn.MaxPool2d(kernel_size=2),
            # output shape (32, 13, 13)

            # input shape (32, 13, 13)
            nn.Conv2d(32, 64, 3, 1, 0),
            # output shape (64, 11, 11)
            nn.ReLU(),  # activation
            nn.MaxPool2d(2),
            # output shape (64, 5, 5)

            # input shape (64, 5, 5)
            nn.Conv2d(64, 64, 3, 1, 0),
            # output shape (64, 3, 3)
            nn.ReLU()  # activation
            # output shape (64, 3, 3)
        )        

        self.conv2 = nn.Sequential(  
            # input shape (1, 28, 28)
            nn.Conv2d(
                in_channels=1,  # 输入通道数
                out_channels=32,  # 输出通道数
                kernel_size=3,   # 卷积核大小          
                stride=1,  #卷积步数
                padding=0  # 如果想要 con2d 出来的图片长宽没有变化,padding=(kernel_size-1)/2 当 stride=1
            ),
            # output shape (32, 26, 26)
            nn.ReLU(),  # activation
            nn.MaxPool2d(kernel_size=2),
            # output shape (32, 13, 13)

            # input shape (32, 13, 13)
            nn.Conv2d(32, 64, 3, 1, 0),
            # output shape (64, 11, 11)
            nn.ReLU(),  # activation
            nn.MaxPool2d(2),
            # output shape (64, 5, 5)

            # input shape (64, 5, 5)
            nn.Conv2d(64, 64, 3, 1, 0),
            # output shape (64, 3, 3)
            nn.ReLU()  # activation
            # output shape (64, 3, 3)
        )      

        self.conv3 = nn.Sequential(  
            # input shape (1, 28, 28)
            nn.Conv2d(
                in_channels=1,  # 输入通道数
                out_channels=32,  # 输出通道数
                kernel_size=3,   # 卷积核大小          
                stride=1,  #卷积步数
                padding=0  # 如果想要 con2d 出来的图片长宽没有变化,padding=(kernel_size-1)/2 当 stride=1
            ),
            # output shape (32, 26, 26)
            nn.ReLU(),  # activation
            nn.MaxPool2d(kernel_size=2),
            # output shape (32, 13, 13)

            # input shape (32, 13, 13)
            nn.Conv2d(32, 64, 3, 1, 0),
            # output shape (64, 11, 11)
            nn.ReLU(),  # activation
            nn.MaxPool2d(2),
            # output shape (64, 5, 5)

            # input shape (64, 5, 5)
            nn.Conv2d(64, 64, 3, 1, 0),
            # output shape (64, 3, 3)
            nn.ReLU()  # activation
            # output shape (64, 3, 3)
        )      

        self.fc1 = nn.Linear(64*3*3, 1*28*28)
        self.fc2 = nn.Linear(64*3*3, 1*28*28)
        self.fc3 = nn.Linear(64*3*3, 1*28*28)
        self.out = nn.Linear(1*28*28, config['class_num']) 
        
    def forward(self, x):        
        x = self.conv1(x)
        x = x.view(x.size(0), 64*3*3)  # 展平多维的卷积图成 (batch_size, 64 * 3 * 3)
        x = self.fc1(x)

        # x = x.view(x.size(0), 1,28,28)
        # x = self.conv2(x)   
        # x = x.view(x.size(0), 64*3*3)  # 展平多维的卷积图成 (batch_size, 64 * 3 * 3)
        # x = self.fc2(x)    

        # x = x.view(x.size(0), 1,28,28)
        # x = self.conv3(x)   
        # x = x.view(x.size(0), 64*3*3)  # 展平多维的卷积图成 (batch_size, 64 * 3 * 3)
        # x = self.fc3(x)   

        output = self.out(x)        
        return output


def active_f(x):
    return np.maximum(0, x)
def active_f_g(x):
    return (x >= 0).astype(int)

class BP:
    def __init__(self):
        #初始化参数
        input_nodes = 784
        hidden_nodes = 50
        output_nodes = config['class_num']
        learning_rate = 0.01
        self.__weight1 = np.random.normal(0.0, pow(hidden_nodes, -0.5), (hidden_nodes, input_nodes))
        self.__weight2 = np.random.normal(0.0, pow(output_nodes, -0.5), (output_nodes, hidden_nodes))
        self.__learning_rate = learning_rate

    def train(self, t_input, t_target):
        #前向过程
        inputs = np.array(t_input, ndmin=2).T
        targets = np.array(t_target, ndmin=2).T
        hidden_inputs = np.dot(self.__weight1, inputs)
        hidden_outputs = active_f(hidden_inputs)
        final_inputs = np.dot(self.__weight2, hidden_outputs)
        final_outputs = active_f(final_inputs)
        
        #计算误差并反向传播
        output_errors = targets - final_outputs
        hidden_errors = np.dot(self.__weight2.T, output_errors)
        self.__weight2 += self.__learning_rate * np.dot((output_errors * active_f_g(final_inputs)),np.transpose(hidden_outputs))
        self.__weight1 += self.__learning_rate * np.dot((hidden_errors * active_f_g(hidden_inputs)),(np.transpose(inputs)))
 
    def predict(self, inputs_list):
        #预测过程只需要进行一个前向传播即可
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



class Train:
    def __init__(self):
        self.cnn = CNN()
        self.data = getData()

        self.bp = BP()

    def train(self):
        optimizer = torch.optim.Adam(self.cnn.parameters())
        self.cnn.to(device)
        train_images = torch.from_numpy(self.data.train_images).float().to(device)
        test_images = torch.from_numpy(self.data.test_images).float().to(device)
        train_labels = torch.from_numpy(self.data.train_labels).long().to(device)
        test_labels = torch.from_numpy(self.data.test_labels).long().to(device)

        self.cnn.train()
        for local_epoch in range(20):
            print(local_epoch)
            out_tensor = self.cnn(train_images)
            loss = nn.CrossEntropyLoss()(out_tensor, train_labels)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

        torch.save(self.cnn.state_dict(), config['check_path'],_use_new_zipfile_serialization=False)

        self.cnn.eval()
        with torch.no_grad():
            acc = []
            for i in range(len(test_images)):
                label = torch.argmax(self.cnn(test_images[i:i+1])[0])
                #选取概率最大的数字为输出，若和标签相同则预测成功
                if int(test_labels[i]) == label:
                    acc.append(1)
                else:
                    acc.append(0)
            #计算平均准确率
            print("acc is ", np.array(acc).mean())

            begin_t = rospy.Time.now()
            out = self.cnn(test_images[i:i+1])[0]
            print(out)
            end_t = rospy.Time.now()
            print("Duration: {}".format((end_t - begin_t).to_sec()))


    def trainBP(self):
        for epo in range(3):
            print(epo)
            for i in range(len(self.data.train_images)):
                #调用train进行训练
                label = [0,0,0,0]
                label[int(self.data.train_labels[i])] = 1
                self.bp.train(self.data.train_images[i].reshape(784),label)
        self.bp.saveweight(config['BP_path'])
        acc = []
        for i in range(len(self.data.test_images)):
            label = np.argmax(self.bp.predict(self.data.test_images[i].reshape(784)))
            #选取概率最大的数字为输出，若和标签相同则预测成功
            if int(self.data.test_labels[i]) == label:
                acc.append(1)
            else:
                acc.append(0)
        #计算平均准确率
        print("acc is ", np.array(acc).mean())
        begin_t = rospy.Time.now()
        Num = self.bp.predict(np.array([1]*28*28*1).reshape(784))
        print(Num)
        end_t = rospy.Time.now()
        print("Duration: {}".format((end_t - begin_t).to_sec()))



if __name__ == "__main__":
    rospy.init_node('train')
    trainer = Train()
    trainer.train()
    # trainer.trainBP()