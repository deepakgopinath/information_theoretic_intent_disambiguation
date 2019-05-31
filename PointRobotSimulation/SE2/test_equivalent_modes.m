clear all; 
%%

cm = {[1,2],3, 4};
EID_AR = [-1.033, -1.033, -1.1]';
num_modes = length(EID_AR); 
thresh = 10^-3;
diff_p = squareform(pdist(EID_AR)/max(abs(EID_AR)));
inds = find(diff_p < thresh);
eq_modes = {};
if length(inds) > num_modes
    [i,j] = ind2sub([num_modes, num_modes], inds);
    eq_ind = i== j;
    i(eq_ind) = []; j(eq_ind) = [];
    eq_modes = unique(i+j);
    eq_modes = [i,j]; eq_modes = num2cell(eq_modes, 2);
    flag = true;
    index  = 1; offset = 1;
    while flag
        if index == length(eq_modes)
            break
        end
        if ~isempty(intersect(eq_modes{index}, eq_modes{index+offset}))
            eq_modes{index} = union(eq_modes{index}, eq_modes{index + offset});
            eq_modes(index + offset) = [];
        else
            offset = offset  + 1;
            if index + offset > length(eq_modes)
                index = index + 1;
                offset = 1;
            end
        end
    end
end


if ~isempty(eq_modes) %there are equivalent modes. 
    if length(eq_modes) == 1 &&  length(eq_modes{1}) == length(cm)
        fprintf('All modes disambiguate equally. Dont change current mode \n');
    elseif length(eq_modes) == 1 && length(eq_modes{1}) < length(cm)
        non_equiv_modes = setdiff(1:length(cm), eq_modes{1});
        best_mode = find(EID_AR == max(EID_AR));
        if ~isempty(find(best_mode == eq_modes{1}, 1))
            fprintf('the following modes disambiguate the best and equally\n');
            for i=1:length(eq_modes{1})
                fprintf('Mode %d\n',eq_modes{1}(i));
                disp(cm{eq_modes{1}(i)});
            end
        else
            fprintf('The best mode is %d\n', cm{best_mode});
        end
    end
else
    best_mode = find(EID_AR == max(EID_AR));
    fprintf('The best mode is %d\n', cm{best_mode});
end

