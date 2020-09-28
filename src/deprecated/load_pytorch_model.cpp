#include <torch/script.h> // One-stop header.

#include <iostream>
#include <memory>
#include <chrono> 

using namespace std::chrono; 

int main(int argc, const char* argv[]) {
    if (argc != 2) {
        std::cerr << "usage: example-app <path-to-exported-script-module>\n";
        return -1;
    }


    torch::jit::script::Module module;
    try {
        // Deserialize the ScriptModule from a file using torch::jit::load().
        module = torch::jit::load(argv[1]);
    }
    catch (const c10::Error& e) {
        std::cerr << "error loading the model\n";
        return -1;
    }

    std::cout << "ok\n";

    // Create a vector of inputs.
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(-3.1415+2*3.1415*torch::rand({4}));
    // Execute the model and turn its output into a tensor.
    at::Tensor output = module.forward(inputs).toTensor();
    //at::Tensor jacobian = module.jacobian(inputs).toTensor();
    std::cout << "Out : " << output << '\n';
    std::cout << "Jac : " << output << '\n';


    // Function evaluation with start and stop timestamps
    auto start_cg = high_resolution_clock::now();
    
    for (int k = 0; k<1e3; k++)
    {
        // Create a vector of inputs.
        inputs[0] = (-3.1415+2*3.1415*torch::rand({4}));
        // Execute the model and turn its output into a tensor.
        output = module.forward(inputs).toTensor();
        //jacobian = module.jacobian(inputs).toTensor();
        //std::cout << output << '\n';

    }

    

    auto stop_cg = high_resolution_clock::now(); 
    auto duration_cg = duration_cast<microseconds>(stop_cg - start_cg); 

    std::cout << "\nTime taken by function (1e3 executions): " << ((int)duration_cg.count()) << " microseconds" << std::endl; 
}