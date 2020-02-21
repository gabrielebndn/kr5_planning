function v_sc=saturate(v,v_max)
    v_sc=v;
    for k=1:size(v)
        if v(k)>v_max(k)
            v_sc(k)=v_max(k);
        elseif v(k)<-v_max(k)
            v_sc(k)=-v_max(k);
        end
    end
            