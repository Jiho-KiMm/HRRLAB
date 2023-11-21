import socket
import time
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque
from torch.autograd import Variable
import csv
import time

file_name = "Take42.csv"
device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f'device = {device}')

flag = True

input_data_deque = deque()
pred_z = []
Fz = []
t = []
predicted = 0
contact = 0


seq_length = 7
data_dim = 6
hidden_dim = 10
output_dim = 1
learning_rate = 0.01
epochs = 1000
lstm_layers = 3


class Net(nn.Module):
    def __init__(self, input_dim, hidden_dim, sequence_len, output_dim, layers):
        super(Net, self).__init__()
        self.hidden_dim = hidden_dim
        self.seq_len = sequence_len
        self.output_dim = output_dim
        self.layers = layers

        self.lstm = nn.LSTM(input_dim, hidden_dim, num_layers=layers, batch_first=True)
        self.fc = nn.Linear(hidden_dim, output_dim, bias=True)

    # 학습 초기화를 위한 함수
    def reset_hidden_state(self):
        self.hidden = (torch.zeros(self.layers, self.seq_len, self.hidden_dim),
                       torch.zeros(self.layers, self.seq_len, self.hidden_dim))

    def forward(self, x):
        h_0 = Variable(torch.zeros(self.layers, x.size(0), self.hidden_dim)).to(device)  # hidden state
        c_0 = Variable(torch.zeros(self.layers, x.size(0), self.hidden_dim)).to(device)  # internal state

        output, (hn, cn) = self.lstm(x, (h_0, c_0))  # lstm with input, hidden, and internal state
        out = self.fc(output[:, -1])  # shape --> (32, 3)
        return out


model = Net(data_dim, hidden_dim, seq_length, output_dim, lstm_layers).to(device)
model.load_state_dict(torch.load('Lstm_take1_weights.pth'))


def receive_continuous_data():
    global t, predicted, contact
    # 서버의 IP 주소와 포트 번호

    # 소켓 생성

    try:
        # 서버에 연결

        server_ip = "192.168.137.10"
        server_port = 9001

        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        client_socket.connect((server_ip, server_port))

        received_data = client_socket.recv(1024).decode('utf-8')  # 1024 바이트씩 데이터를 받습니다. 필요에 따라 조절할 수 있습니다.

        # 수신한 데이터를 ":"로 구분하여 리스트로 만듭니다.
        data_list = received_data.split(":")
        # print(f'received_data = {received_data}')

        input_data_set = [float(data_list[0]), float(data_list[1]), float(data_list[2]), float(data_list[3]),
                          float(data_list[4]), float(data_list[5])]

        # ttt = [float(data_list[0]), float(data_list[1]), float(data_list[2]), float(data_list[3]), float(data_list[4]),
        #        float(data_list[5]), float(data_list[6])]
        #
        # t.append(ttt)

        # print(f'input_data_set = {input_data_set}')
        if len(input_data_deque) == 7 and float(data_list[4]) > 0:
            input_data_deque.popleft()
            input_data_deque.append(input_data_set)
            input_tensor = torch.FloatTensor(input_data_deque).to(device)
            input_tensor = input_tensor.unsqueeze(0)
            with torch.no_grad():
                predicted = model(input_tensor)
                pred_z.append(predicted)

        if len(input_data_deque) < 7 and float(data_list[4]) > 0:
            input_data_deque.append(input_data_set)
            pred_z.append(0)
            predicted = 0
            if len(input_data_deque) == 7:
                input_tensor = torch.FloatTensor(input_data_deque).to(device)
                input_tensor = input_tensor.unsqueeze(0)
                with torch.no_grad():
                    predicted = model(input_tensor)
                    pred_z.append(predicted)

        if len(input_data_deque) == 7 and float(data_list[4]) > 0:
            Fz.append(float(data_list[6]))

        if predicted > 15:
            contact = 1
        elif predicted <= 15:
            contact = 0

        if float(data_list[4]) > 0:
            print(f'contact = {contact}')

        client_socket.close()

    except Exception as e:
        data = list(zip(pred_z, Fz))
        with open(file_name, 'w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)

            csv_writer.writerow(["pred", "Fz"])
            csv_writer.writerows(data)


        # data = list(zip(pred_z, Fz))
        # with open(file_name, 'w', newline='') as csv_file:
        #     csv_writer = csv.writer(csv_file)
        #
        #     csv_writer.writerow("t")
        #     csv_writer.writerows(t)

        print(f'pred {pred.shape}.')
        print(f'Fz {error.shape}.')
        print(f'Data has been exported to {file_name}.')
        print("Error:", e)
        flag = False


    finally:

        client_socket.close()


if __name__ == "__main__":

    while flag:
        receive_continuous_data()
