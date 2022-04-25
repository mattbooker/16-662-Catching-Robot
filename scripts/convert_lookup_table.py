import pickle

table = pickle.load(open("lookup_table_5cm.p", "rb"))

a = list(table.keys())[0]
print(table[a].keys())

# Round raw keys
# table = pickle.load(open("lookup_table_5cm_raw.p", "rb"))

# new_table = defaultdict(dict)

# for x in table.keys():
#     for y in table[x].keys():
#         saved_x = np.round(0.05 * np.round(x / 0.05), 2)
#         saved_y = np.round(0.05 * np.round(y / 0.05), 2)
        
        
#         new_table[saved_x][saved_y] = table[x][y]

# pickle.dump(new_table, open("lookup_table_5cm.p", "wb"))