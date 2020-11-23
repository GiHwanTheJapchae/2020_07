import temp_comm

def get_location():
    connection = comm('127.0.0.1', 8080)
    geolist = connection.conn_getd()
    for i in range(len(geolist)):
        if i % 2 == 0:
            latitude = locations[i]
    else:
        longtitude = locations[i]
        connection.conn_ed()
