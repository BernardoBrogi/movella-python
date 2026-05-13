#%%
from movella_streamer_class import MovellaStreamer  # your online class, no UDP

tracker = MovellaStreamer("config.json", "UNISI", n_trackers=4, udp_ip="127.0.0.1", udp_port=8051)

tracker.initialize()
#%%
tracker.stream_udp_loop()
#%% 
# tracker.calibrate()
#%%
# tracker.compute_kernel(send_ip="172.16.0.1",send_port=8052) 
#%%
tracker.cleanup()
# %%
