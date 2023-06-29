import pickle

sample_num = 0

# pkl_file_1 = "data/v1.0-mini/v1.0-mini_infos_train.pkl"
# with open(pkl_file_1, 'rb') as file:
#     data = pickle.load(file)
# for item in data['data_list'][sample_num]:
#     print(item, data['data_list'][sample_num][item])
#
# print(f'\n')
#
# pkl_file_2 = "data/mini_set_1/mini_set_1_infos_train.pkl"
# with open(pkl_file_2, 'rb') as file:
#     data = pickle.load(file)
# for item in data['data_list'][sample_num]:
#     print(item, data['data_list'][sample_num][item])
#
#
# pkl_file = "data/v1.0-mini/v1.0-mini_dbinfos_train.pkl"
# with open(pkl_file, 'rb') as file:
#     data = pickle.load(file)
# for item in data:
#     print(item)
# print(data['car'][:20])
#
# pkl_file = "data/mini_set_1/mini_set_1_dbinfos_train.pkl"
# with open(pkl_file, 'rb') as file:
#     data = pickle.load(file)
# for item in data:
#     print(item)
# print(data['car'])

# pkl_file = "data/v1.0-mini/v1.0-mini_infos_train.pkl"
# pkl_file = "data/mini_set_3_working/mini_set_3_infos_train.pkl"
# pkl_file = r"C:\Users\malak\Documents\MasterThesis\MasterThesis\mmdetection3d\tools\mini_set_3_th3.pkl"
# pkl_file = r"C:\Users\malak\Documents\MasterThesis\MasterThesis\mmdetection3d\data\horus_infos_val_range_0-10.pkl"
pkl_file = "/home/marcus/MasterThesis/TransFusion/tools/horus_infos_val_range_0-10.pkl"

with open(pkl_file, 'rb') as file:
    data = pickle.load(file)
    print(data)
    # print(data['metainfo'])

for key, value in data['data_list'][0].items():
    print(key, value)

# print(data['data_list'][0])
# for item in data:
#     print(item)
# print(data['car'])

