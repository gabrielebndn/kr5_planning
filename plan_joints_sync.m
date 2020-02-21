function q=plan_joints_sync(q_init,q_end,t,v_max,a_max)

q = repmat(q_init,1,length(t)) + repmat(sign(q_end-q_init),1,length(t)).*bang_bang_sync(abs(q_end-q_init),t,v_max,a_max);