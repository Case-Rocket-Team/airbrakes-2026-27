load("CdA_function_data.mat")

X_test = [449.0790; -214.3062];

CdA_test = CdA_calculator(X_test, F_CdAs, CdAs)
