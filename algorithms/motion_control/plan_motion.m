function public_vars = plan_motion(read_only_vars, public_vars)

persistent step_counter seq_idx


if isempty(step_counter) || read_only_vars.counter == 1
    step_counter = 0;
    seq_idx = 1;
end


sequence = [
    0.99,  1,  70;  
    0.8,  1,  15;  
    1.0,  1.0,  10;  
    0.8,  1,  15; 
    1, 1,  60;  
    1.0,  0.8,  15;  
    1.0,  1.0,  10; 
    1.0,  0.8,  15;  
    1,  1,  999  
];

step_counter = step_counter + 1;


if step_counter > sequence(seq_idx, 3)
    
    seq_idx = min(seq_idx + 1, size(sequence, 1));
    step_counter = 1; 
end


public_vars.motion_vector = sequence(seq_idx, 1:2);

end