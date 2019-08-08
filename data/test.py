
import scipy.io as sio
load_fn='data.mat'
load_data = sio.loadmat(load_fn)
# print(load_data)
print(load_data['room_plane_base_endpoint_cell'][0][0][0][0])