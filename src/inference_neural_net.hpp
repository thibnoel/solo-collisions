#include <iosfwd>
#include "codegen_helper.hpp"


template<typename Scalar>
struct Layer
{
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> weights;
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> bias; 

    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> evaluate(Eigen::Matrix<Scalar, Eigen::Dynamic, 1> input)
    {
        return weights*input + bias;
    }
};


template<typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> tanhActGrad(Eigen::Matrix<Scalar, Eigen::Dynamic, 1> input)
{
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> diag;
    diag = (Scalar)1 - Eigen::tanh(input.array()).array()*Eigen::tanh(input.array()).array();
    return diag.matrix().asDiagonal();
}


// Class for the neural network
template<typename Scalar>
class InferenceNeuralNetwork
{
    public :

    int nbLayers;
    std::vector<Layer<Scalar>> layers;

    // Converts q to x = [cos(q0),...,cos(qn),sin(q0),...,sin(qn)], 
    // suitable input for the neural network.
    // Actually not needed for the code generation (tracing the operations on a random x is enough)
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> qToInput(Eigen::Matrix<Scalar, Eigen::Dynamic, 1> q)
    {
        //return input;
        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> net_input;
        int n = (int)q.rows();
        net_input.resize(2*n,1);

        net_input.block(0,0,n,1) = Eigen::cos(q.array());
        net_input.block(n,0,n,1) = Eigen::sin(q.array());
        
        return net_input;
    }


    // Applies the feed forward pass with a tanh activation
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> feedForward(Eigen::Matrix<Scalar, Eigen::Dynamic, 1> input)
    {
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> input_copy;
        input_copy = input;

        for(unsigned int k=0; k<this->layers.size(); k++)
        {
            // Evaluate W*x + b
            Eigen::Matrix<Scalar, Eigen::Dynamic, 1> int_res;
            int_res = this->layers[k].evaluate(input_copy); 

            if(k < this->layers.size() - 1)
            {
                // Evaluate tanh activation
                int_res = Eigen::tanh(int_res.array());
            }
            input_copy.resize(int_res.size(),1);
            input_copy = int_res;
        }
        return input_copy;
    }

    // Computes the network gradient
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> outInGradient(Eigen::Matrix<Scalar, Eigen::Dynamic, 1> input)
    {
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> input_copy;
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> jac;
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> int_jac;

        input_copy = input;
        jac = this->layers[0].weights;

        for(unsigned int k=0; k<this->layers.size() - 1; k++)
        {
            
            Eigen::Matrix<Scalar, Eigen::Dynamic, 1> int_res;
            int_res = this->layers[k].evaluate(input_copy); 

            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> actGrad;
            actGrad = tanhActGrad(int_res);
            
            int_jac.resize(actGrad.rows(), jac.cols());
            int_jac = actGrad*jac;

            jac.resize(this->layers[k + 1].weights.rows(), int_jac.cols());
            jac = this->layers[k + 1].weights*int_jac;

            input_copy.resize(int_res.size(), 1);
            input_copy = int_res;
        }

        return jac;
    }

    // Computes the 'input' gradient J = [dxj/dqi]
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> inQGradient(Eigen::Matrix<Scalar, Eigen::Dynamic, 1> input)
    {
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Jinq;
        int n = (int)input.rows()/2;
        Jinq.resize(n*2,n);
        
        Jinq.block(0,0,n,n) = -(Scalar)1.0*input.block(n,0,n,1).asDiagonal();
        Jinq.block(n,0,n,n) = input.block(0,0,n,1).asDiagonal();

        return Jinq;
    }

};

//ADFun tapeADNeuralNetInference(InferenceNeuralNetwork<ADScalar> nn)
ADFun tapeADNeuralNetInference(InferenceNeuralNetwork<ADScalar> nn, const int q_input_size)
{
    // Initnialize AD input and output
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_X;
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_Y;
    // Input size = nb_DoF * 2
    ad_X.resize(2*q_input_size);
    //ad_X << (ADScalar)0, (ADScalar)0, (ADScalar)0, (ADScalar)0;
    // Output size = 1 + input size/2 (dist + jac)
    ad_Y.resize(1+q_input_size);
    CppAD::Independent(ad_X);
    // Initialize AD function
    ADFun ad_fun;   

    ad_Y[0] = nn.feedForward(ad_X)[0];
    Eigen::Matrix<ADScalar, Eigen::Dynamic, Eigen::Dynamic> jac_net = nn.outInGradient(ad_X);
    Eigen::Matrix<ADScalar, Eigen::Dynamic, Eigen::Dynamic> jac_inq = nn.inQGradient(ad_X);

    //ad_Y.block<2,1>(1,0) = (jac_net*jac_inq).transpose();
    ad_Y.block(1,0, q_input_size, 1) = (jac_net*jac_inq).transpose();

    ad_fun.Dependent(ad_X, ad_Y);

    return ad_fun;
}