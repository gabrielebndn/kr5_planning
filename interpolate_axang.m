function R=interpolate_axang(s,R_init,r,D_theta)

R=R_init*angvec2r(s*D_theta,r);