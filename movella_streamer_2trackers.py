
#%%
from movella_streamer_class import MovellaStreamer  # your online class, no UDP

tracker = MovellaStreamer("config.json", "UNISI", n_trackers=2, udp_ip="127.0.0.1", udp_port=8051)

tracker.initialize()
 #%%
tracker.calibrate_2arm(imu="arm")
 #%%
tracker.calibrate_2arm(imu="forearm")
#%%
tracker.compute_easy_kernel(send_ip="172.16.0.1",send_port=8052)
#%%
tracker.cleanup()
# %%
