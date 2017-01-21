function [  ] = autofunc( path, template, I, O )
%AUTOFUNC 
%   

%% 
import ddp.dynamics.sym2exp

top = 'function [';
for i = 1:length(template{1})
    top = [top template{1}{i}];
    if i < length(template{1})
        top = [top ', '];
    end
end
top = [top '] = ' template{2}{1} '('];
for i = 1:length(template{3})
    top = [top template{3}{i}];
    if i < length(template{3})
        top = [top ', '];
    end
end
top = [top ')' char(10)];
body = '';
for i = 1:length(template{3})
    if isfield(I,template{3}{i})
        fn = template{3}{i};
        f = getfield(I,fn);
        for j = 1:length(f)
            body = [body char(f(j)) ' = ' fn '(' num2str(j) ');' char(10)];
        end
    end
end
for i = 1:length(template{1})
    fn = template{1}{i};
    f = getfield(O,fn);
    body = [body fn ' = ' sym2exp(f) char(10)];
end
bot = ['end' char(10)];
funcstr = [top body bot]

fid = fopen(path,'w');
fprintf(fid,'%s',funcstr);
fclose(fid);

end

