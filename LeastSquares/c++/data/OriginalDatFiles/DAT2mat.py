import scipy.io as sio
import glob
from tqdm import tqdm
files = glob.glob("*.dat")

for f in files:
    data = sio.loadmat(f)
    sio.savemat(f.replace(".dat",".mat"),data)
