function result = apply(fn, X)
result = cellfun(fn, num2cell(X', 1))';
end