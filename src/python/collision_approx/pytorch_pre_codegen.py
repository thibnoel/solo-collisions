from collision_approx.collision_approx_pytorch import *

# Generates C++ code for a PyTorch model
# This C++ code can then be compiled and executed to generate C source code for inference
# Basically, here is a bridge from python to CppAD::codegen, based on the implementation provided in inference_neural_net.hpp

def generateWarning():
    warning = "/" + 50*"*"
    warning += "\n This code was generated ! \nSee the reference example under inference_neural_net_codegen.cpp\n"
    warning += 50*"*"
    warning += '/\n\n'
    return warning

def generateHeaders():
    headers = \
'#include <stdlib.h> \n\
#include <iostream> \n\
#include <fstream>  \n\
#include "inference_neural_net.hpp"\n\
#include <chrono> \n\
#include <sstream>\n\
\
using namespace std::chrono;\n'
    return headers

def generateEigenMat(dim, name, mat_type='double', indent=''):
    mat_string = '{}Eigen::Matrix<{}, {}, {}> {};\n'.format(indent, mat_type, dim[0], dim[1], name)
    return mat_string


def generateMatInit(dim, name, values_array, indent=''):
    init_string = '{}{} << '.format(indent, name)
    for i in range(dim[0]):
        val_line = '' if i==0 else '{}  '.format(indent)
        for j in range(dim[1]):
            sep = ',' if (i< dim[0]-1 or j<dim[1]-1) else ';'
            val_str = str(values_array[j,i]) if dim[1] > 1 else str(values_array[i])
            val_line = val_line + val_str + sep + ' '
        val_line += '\n'
        init_string += val_line
    return init_string


def generateLayer(name, weights_name, bias_name, layer_type='double', indent=''):
    layer_string = '{}Layer<{}> {} = {{ {}, {} }};\n'.format(indent, layer_type, name, weights_name, bias_name)
    return layer_string


def generateNeuralNet(name, layers_names_list, nn_type='double', indent=''):
    nn_string = '{}InferenceNeuralNetwork<{}> {};\n'.format(indent, nn_type,name)
    for k in range(len(layers_names_list)):
        nn_string = nn_string + "{}{}.layers.push_back({});\n".format(indent, name, layers_names_list[k])
    return nn_string


def generateCGLib(model_name, func_name, lib_name, q_dim, indent=''):
    string = '{}ADFun {} = tapeADNeuralNetInference({}, {});\n'.format(indent, func_name, model_name, int(q_dim))
    string += '{}generateCompileCLib("{}", {});\n'.format(indent, lib_name, func_name)
    return string


def generateCGMain(model_path, model_arch, model_name):
    model = loadTrainedNeuralNet(model_path, model_arch)
    params_list = model.getParamsValues()

    nb_layers = len(model_arch)
    weights = []
    biases = []

    for k in range(nb_layers):
        weights.append(params_list[2*k][1].numpy())
        biases.append(params_list[2*k+1][1].numpy())
    
    main_str = generateWarning()
    main_str += generateHeaders()
    main_str += '\nint main()\n'
    main_str += '{\n'

    for k in range(nb_layers):
        w_str = generateEigenMat([model_arch[k][1], model_arch[k][0]], 'w{}'.format(k), indent='    ')
        b_str = generateEigenMat([model_arch[k][1],1], 'b{}'.format(k), indent='    ')

        main_str = main_str + w_str + b_str
    main_str += '\n'
    for k in range(nb_layers):
        w_init_str = generateMatInit([model_arch[k][1], model_arch[k][0]], 'w{}'.format(k), weights[k].T, indent='    ')
        b_init_str = generateMatInit([model_arch[k][1], 1], 'b{}'.format(k), biases[k].T, indent='    ')
    
        main_str = main_str + '\n' + w_init_str + b_init_str
    main_str += '\n'
    for k in range(nb_layers):
        layer_str = generateLayer('layer{}'.format(k), 'w{}'.format(k), 'b{}'.format(k), layer_type='ADScalar', indent='    ')
    
        main_str += layer_str
    main_str += '\n'
    nn_str = generateNeuralNet(model_name, ['layer{}'.format(k) for k in range(nb_layers)], nn_type='ADScalar', indent='    ')
    main_str += nn_str
    main_str += '\n'

    cg_str = generateCGLib(model_name, 'generatedModel', 'codegen_{}'.format(model_name), model_arch[0][0]/2, indent='    ')
    main_str += cg_str

    main_str += '\n'
    main_str += '}'

    return main_str


def writeFile(path, content_string):
    text_file = open(path, "w")
    text_file.write(content_string)
    text_file.close()


if __name__ == "__main__":
    trainedModel_path = "/home/tnoel/npy_data/pytorch_data/test_2Dmodel_tanh_461.pth"
    trained_model_arch = [[4,8],[8,1]]
    #shoulder_model = loadTrainedNeuralNet(trainedModel_path, trained_model_arch)

    s = generateCGMain(trainedModel_path, trained_model_arch, 'test_model')