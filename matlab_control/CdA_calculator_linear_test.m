clear;
clc;
close all;

X_test = [449.0790; -214.3062];

load("CdA_calculator_linear_data.mat")

CdA = CdA_calculator_linear(X_test, CdAs, Xs_rows, Vs_rows);
