function v = vers(x)
	% vers  Versore (vettore unitario) di x.
	%       Restituisce il vettore nullo se norm(x) e' trascurabile,
	%       per evitare divisioni per zero / NaN.
	%
	% Input  : x  (vettore Nx1 o 1xN)
	% Output : v  (versore, stessa forma di x)

	n = norm(x);
	if n > 1e-12
		v = x / n;
	else
		v = zeros(size(x));
	end
end
