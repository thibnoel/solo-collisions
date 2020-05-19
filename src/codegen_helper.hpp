#include <pinocchio/codegen/cppadcg.hpp>

#ifndef __CODEGEN_HELPER_HPP
#define __CODEGEN_HELPER_HPP

// Code gen. specific types
typedef double Scalar;

typedef CppAD::cg::CG<Scalar> CGScalar;
typedef CppAD::AD<CGScalar> ADScalar;
typedef CppAD::ADFun<CGScalar> ADFun;

// Takes in an ADFun reference (already taped)
// and the input vector size, returns the generated C code as a string
std::string generateCSourceCode(ADFun& adFun, int xSize)
{
    /***************************************************************************
     *                        Generate the C source code
     **************************************************************************/
    CppAD::cg::CodeHandler<Scalar> handler;

    CppAD::vector<CGScalar> indVars(xSize); // size of x (Independent variable)
    handler.makeVariables(indVars);

    CppAD::vector<CGScalar> gen_fun = adFun.Forward(0, indVars);

    // ??? Set parameters for code generation?
    CppAD::cg::LanguageC<Scalar> langC("double");
    CppAD::cg::LangCDefaultVariableNameGenerator<Scalar> nameGen;

    // Generate function code
    std::ostringstream code;
    handler.generateCode(code, langC, gen_fun, nameGen);

    return code.str();
}

// Takes in a taped ADFun and its name as a string, to generate and compile 
// it on the fly under 'libCGfunc_name.so'
void generateCompileCLib(std::string func_name, ADFun& fun)
{
    using namespace CppAD;
    using namespace CppAD::cg;
    /***************************************************************************
     *                        Other method : create and compile
     *                           the C code on the flight
     **************************************************************************/
    std::string lib_name = "libCG" + func_name;
    // Initialize library
    ModelCSourceGen<Scalar> cgen(fun, func_name);
    cgen.setCreateForwardZero(true); // generates the function 
    cgen.setCreateJacobian(false); // generates the jacobian

    ModelLibraryCSourceGen<Scalar> libcgen(cgen);
    DynamicModelLibraryProcessor<Scalar> dynamicLibManager(libcgen, lib_name);

    // Compile library
    GccCompiler<Scalar> compiler;
    std::vector<std::string> compile_options = compiler.getCompileFlags();
    compile_options[0] = "-Ofast";
    compiler.setCompileFlags(compile_options);
    dynamicLibManager.createDynamicLibrary(compiler, false);
    //std::unique_ptr<DynamicLib<Scalar> > dynamicLib;

    // Load library 
    /*
    const auto it = dynamicLibManager.getOptions().find("dlOpenMode");
    if(it == dynamicLibManager.getOptions().end()){
        dynamicLib.reset(new LinuxDynamicLib<Scalar>(dynamicLibManager.getLibraryName() + cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION));
    } else {
        int dlOpenMode = std::stoi(it->second);
        dynamicLib.reset(new LinuxDynamicLib<Scalar>(dynamicLibManager.getLibraryName() + cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION, dlOpenMode));
    }
    
    std::unique_ptr<GenericModel<Scalar> > genFun_ptr = dynamicLib->model(func_name.c_str());*/
}

#endif