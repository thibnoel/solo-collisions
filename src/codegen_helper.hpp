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

void generateCompileCLib();

#endif