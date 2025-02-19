function output = evaluateFuzzyLogic(input1, input2)
    % Load the fuzzy system from the .fis file
    fis = readfis('C:\Users\xyz\Desktop\MATLABFIS\my_fuzzy_logic.fis');
    
    % Evaluate the fuzzy logic system with the given inputs
    output = evalfis([input1, input2], fis);
end
