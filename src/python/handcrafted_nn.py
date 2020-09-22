import numpy as np


class Layer():
    def __init__(self, arrayW, arrayB):
        self.weights = arrayW
        self.biases = arrayB
    
    def eval(self, input):
        return self.weights@input + self.biases


class Network():
    def __init__(self, activation):
        self.activation = activation
        self.layers = []

    def feedforward(self, input):
        for k in range(len(self.layers)):
            input =  self.layers[k].eval(input)

            if(k < len(self.layers) - 1):
                input = self.activation(input)
        return input

    def tanhGrad(self, input):
        return np.diag(1 - np.tanh(input)*np.tanh(input))

    def inputGrad(self, input):
        J = self.layers[0].weights

        for k in range(len(self.layers) - 1):
            input =  self.layers[k].eval(input)
            actGrad = self.tanhGrad(input)

            J = actGrad@J
            J = self.layers[k + 1].weights@J
        #J = self.layers[-1].weights@J
        return J
            
'''
w0 = np.random.random((5,8))
b0 = np.random.random((5))

w1 = np.random.random((2,5))
b1 = np.random.random((2))

net = Network(np.tanh)
net.layers.append(Layer(w0,b0))
net.layers.append(Layer(w1,b1))

test_in = np.random.random(8)
'''

