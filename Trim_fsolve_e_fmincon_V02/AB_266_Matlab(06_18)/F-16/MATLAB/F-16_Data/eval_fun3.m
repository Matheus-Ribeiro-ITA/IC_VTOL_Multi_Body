function [X, Y, Z] = eval_fun3(fun, x, y, z)

sx = length(x);
sy = length(y);
sz = length(z);
if strcmp(z,'~')
    x = x(:);
    y = y(:);
    [X,Y] = meshgrid(x,y);

    i_max = size(X,1);
    j_max = size(X,2);
    Z = zeros(i_max, j_max);
    for i = 1 : i_max
        for j = 1 :j_max
            Z(i,j) = fun(X(i,j), Y(i,j));
        end
    end
else
    auxmin = min([sx,sy,sz]);
    if auxmin > 1
        disp('Uma das entradas tem que ser de dimensão 1');
    else
        auxpos = find([sx,sy,sz]==auxmin);
        switch auxpos
            case 1
                y = y(:);
                z = z(:);
                [X,Y] = meshgrid(y,z);

                i_max = size(X,1);
                j_max = size(X,2);
                Z = zeros(i_max, j_max);
                for i = 1 : i_max
                    for j = 1 :j_max
                        Z(i,j) = fun(x, X(i,j), Y(i,j));
                    end
                end
            case 2
                x = x(:);
                z = z(:);
                [X,Y] = meshgrid(x,z);

                i_max = size(X,1);
                j_max = size(X,2);
                Z = zeros(i_max, j_max);
                for i = 1 : i_max
                    for j = 1 :j_max
                        Z(i,j) = fun(X(i,j), y, Y(i,j));
                    end
                end
            case 3
                x = x(:);
                y = y(:);
                [X,Y] = meshgrid(x,y);

                i_max = size(X,1);
                j_max = size(X,2);
                Z = zeros(i_max, j_max);
                for i = 1 : i_max
                    for j = 1 :j_max
                        Z(i,j) = fun(X(i,j), Y(i,j), z);
                    end
                end
        end                
    end
end

end

