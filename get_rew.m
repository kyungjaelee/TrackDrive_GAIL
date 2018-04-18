function [rew] = get_rew(inval,lrew)
h1_kernel = lrew.h1_kernel;
h1_bias = lrew.h1_bias;
h2_kernel = lrew.h2_kernel;
h2_bias = lrew.h2_bias;
output_kernel = lrew.output_kernel;
output_bias = lrew.output_bias;
n = size(inval,1);

l_hmdn.hid1actv = 'tanh';
switch l_hmdn.hid1actv
    case 'relu'
        h1 = actv_relu(inval*h1_kernel + repmat(h1_bias, n, 1));
    case 'tanh'
        h1 = actv_tanh(inval*h1_kernel + repmat(h1_bias, n, 1));
    case 'elu'
        h1 = actv_elu(inval*h1_kernel + repmat(h1_bias, n, 1));
end
l_hmdn.hid2actv = 'tanh';
switch l_hmdn.hid2actv
    case 'relu'
        h2 = actv_relu(h1*h2_kernel + repmat(h2_bias, n, 1));
    case 'tanh'
        h2 = actv_tanh(h1*h2_kernel + repmat(h2_bias, n, 1));
    case 'elu'
        h2 = actv_elu(h1*h2_kernel + repmat(h2_bias, n, 1));
end
% MIXTURE WEIGHTS
output = h2*output_kernel + repmat(output_bias, n, 1);
rew = -log(actv_sigmoid(output));

end

% ========= ACTIVATION FUNCTIONS =========
% SIGMOID FUNCTION
function out = actv_sigmoid(in)
out = 1./ (1+exp(-in));
end
% TANH FUNCTION
function out = actv_tanh(in)
out = tanh(in);
end
% RELU FUNCTION
function out = actv_relu(in)
out = max(in, 0);
end