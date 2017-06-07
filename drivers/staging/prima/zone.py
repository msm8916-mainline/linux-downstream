class Zone:
    def __init__(self, zone_name = "", country_list=[], db_string=""):
        self.name = zone_name
        self.country_list = country_list
        self.db_string = db_string

    def write_file(self, f):
        f.write("#\n")
        f.write("# Domain : %s\n" % self.name)
        f.write("#\n")
        for country in self.country_list:
            f.write("country %s:\n" % country)
            f.write(self.db_string)
            f.write("\n")
            f.write("\n")

def build_db():
    zone_list = []

    #FCC
    db_string = """\
    (2402 - 2472 @ 40), (N/A, 23)
    (5170 - 5250 @ 80), (N/A, 17)
    (5250 - 5330 @ 80), (N/A, 23), DFS
    (5490 - 5590 @ 80), (N/A, 23), DFS
    (5650 - 5710 @ 40), (N/A, 23), DFS
    (5735 - 5835 @ 80), (N/A, 23)"""
    zone = Zone("FCC",
                ["AS", "AI", "BS", "BM", "CA", "KY", "GU", "FM", "MP", "PR", "US", "UM", "VI"],
                db_string)

    zone_list.append(zone)

    #ETSI
    db_string = """\
    (2402 - 2482 @ 40), (N/A, 23)
    (5170 - 5250 @ 80), (N/A, 17)
    (5250 - 5330 @ 80), (N/A, 23), DFS
    (5490 - 5710 @ 80), (N/A, 23), DFS"""
    zone = Zone("ETSI",
			['AF', 'AL', 'DZ', 'AD', 'AO', 'AT', 'AZ', 'BY', 'BE', 'BJ', 'BT', 'BA', 'BW', 'BG', 'BF', 'BI', 'CM', 'CV', 'CF', 'TD', 'KM', 'CD', 'CG', 'CI', 'HR', 'CY', 'CZ', 'DK', 'GQ', 'EE', 'ET', 'FK', 'FI', 'FR', 'GF', 'PF', 'TF', 'GA', 'GM', 'GE', 'DE', 'GH', 'GI', 'GR', 'GL', 'GP', 'GN', 'GW', 'HU', 'IS', 'IQ', 'IE', 'IT', 'KZ', 'KS', 'KG', 'LV', 'LS', 'LY', 'LI', 'LT', 'LU', 'MK', 'MG', 'ML', 'MT', 'MQ', 'MR', 'MU', 'YT', 'MD', 'MC', 'ME', 'MS', 'MM', 'NR', 'NL', 'NC', 'NE', 'NO', 'OM', 'PL', 'PT', 'RE', 'RO', 'RW', 'SM', 'ST', 'RS', 'SL', 'SK', 'SI', 'ES', 'SR', 'SZ', 'SE', 'CH', 'TJ', 'TZ', 'TG', 'TR', 'TM', 'TC', 'GB', 'VA', 'ZW'],
                db_string)

    zone_list.append(zone)

    #JAPAN
    db_string = """\
(2402 - 2482 @ 40), (N/A, 23)
(5170 - 5250 @ 80), (N/A, 17)
(5250 - 5330 @ 80), (N/A, 23), DFS
(5490 - 5710 @ 80), (N/A, 23), DFS"""
    zone = Zone("JAPAN",
                ['JP'],
                db_string)

    zone_list.append(zone)

    #WORLD
    db_string = """\
(2402 - 2482 @ 40), (N/A, 23)
(5170 - 5250 @ 80), (N/A, 17)
(5250 - 5330 @ 80), (N/A, 23), DFS
(5490 - 5710 @ 80), (N/A, 23), DFS
(5735 - 5835 @ 80), (N/A, 23)"""
    zone = Zone("WORLD",
			['AG', 'AW', 'BZ', 'BR', 'VG', 'KH', ' CO', 'CR', 'DM', 'DO', 'EC', 'SV', 'GD', 'GT', 'HK', 'LA', 'LB', 'LR', 'MO', 'MW', 'MN', 'MZ', 'NA', 'AN', 'NZ', 'NI', 'PW', 'PY', 'PE', 'PH', 'KN', 'LC', 'MF', 'VC', 'SC', 'SG', 'LK', 'TH', 'TT', 'AE', 'VU', 'VN', 'YE', 'ZM'],
                db_string)

    zone_list.append(zone)

    #N_AMER_EXC_FCC
    db_string = """\
    (2402 - 2482 @ 40), (N/A, 23)
    (5170 - 5250 @ 80), (N/A, 17)
    (5250 - 5330 @ 80), (N/A, 23), DFS"""
    zone = Zone("N_AMER_EXC_FCC",
			['AM', 'IL', 'KW', 'MA', 'SN', 'TN', 'UZ'],
                db_string)
    zone_list.append(zone)

    #APAC
    db_string = """\
    (2402 - 2482 @ 40), (N/A, 23)
    (5170 - 5250 @ 80), (N/A, 17)
    (5250 - 5330 @ 80), (N/A, 23), DFS
    (5735 - 5835 @ 80), (N/A, 23)"""
    zone = Zone("APAC",
			['BH', 'BB', 'BN', 'CL', 'CN', 'EG', 'IN', 'MY', 'MV', 'PA', 'UY', 'VE', 'UA'],
                db_string)
    zone_list.append(zone)

    #KOREA
    db_string = """\
    (2402 - 2482 @ 40), (N/A, 23)
    (5170 - 5250 @ 80), (N/A, 17)
    (5250 - 5330 @ 80), (N/A, 23), DFS
    (5490 - 5630 @ 80), (N/A, 23), DFS
    (5735 - 5815 @ 80), (N/A, 23)"""
    zone = Zone("KOREA",
                ['KR'],
                db_string)
    zone_list.append(zone)

    #HI_5GHZ
    db_string = """\
    (2402 - 2482 @ 40), (N/A, 23)
    (5735 - 5835 @ 80), (N/A, 23)"""
    zone = Zone("HI_5GHZ",
			['BD', 'GY', 'HT', 'HN', 'JM', 'PK', 'QA'],
                db_string)
    zone_list.append(zone)

    #NO_5GHZ
    db_string = """\
    (2402 - 2482 @ 40), (N/A, 23)"""
    zone = Zone("NO_5GHZ",
			['DJ', 'ER', 'SB', 'SO', 'TO', 'TV'],
                db_string)
    zone_list.append(zone)


    #WORLD1
    db_string = """\
    (2402 - 2482 @ 40), (N/A, 20)
    (5170 - 5250 @ 80), (N/A, 17)
    (5250 - 5330 @ 80), (N/A, 23), DFS
    (5490 - 5590 @ 80), (N/A, 23), DFS
    (5650 - 5710 @ 40), (N/A, 23), DFS
    (5735 - 5835 @ 80), (N/A, 23)"""
    zone = Zone("WORLD1",
			['AR', 'AU', 'CX', 'FJ', ' KI', 'MX', 'PG', 'RU'],
                db_string)
    zone_list.append(zone)

    #WORLD3
    db_string = """\
    (2402 - 2482 @ 40), (N/A, 20)
    (5170 - 5250 @ 80), (N/A, 17)
    (5735 - 5835 @ 80), (N/A, 23)"""
    zone = Zone("WORLD3",
                ['JO'],
                db_string)
    zone_list.append(zone)

    #WORLD4
    db_string = """\
    (2402 - 2482 @ 40), (N/A, 23)
    (5735 - 5815 @ 80), (N/A, 23)"""
    zone = Zone("WORLD4",
                ['ID'],
                db_string)
    zone_list.append(zone)

    #WORLD5
    db_string = """\
    (2402 - 2482 @ 40), (N/A, 20)
    (5170 - 5250 @ 80), (N/A, 17)
    (5250 - 5330 @ 80), (N/A, 23), DFS
    (5735 - 5815 @ 80), (N/A, 23)"""
    zone = Zone("WORLD5",
                ['NP'],
                db_string)
    zone_list.append(zone)

    #WORLD6
    db_string = """\
    (2402 - 2482 @ 40), (N/A, 23)
    (5250 - 5330 @ 40), (N/A, 23), DFS
    (5490 - 5710 @ 80), (N/A, 23), DFS
    (5735 - 5835 @ 80), (N/A, 23)"""
    zone = Zone("WORLD6",
                ['NG', 'BO'],
                db_string)
    zone_list.append(zone)

    #WORLD7
    db_string = """\
    (2402 - 2472 @ 40), (N/A, 23)
    (5270 - 5330 @ 40), (N/A, 23), DFS
    (5490 - 5590 @ 80), (N/A, 23), DFS
    (5650 - 5710 @ 40), (N/A, 23), DFS
    (5735 - 5835 @ 80), (N/A, 23)"""
    zone = Zone("WORLD7",
                ['TW'],
                db_string)
    zone_list.append(zone)

    #WORLD10
    db_string = """\
    (2402 - 2482 @ 40), (N/A, 23)
    (5170 - 5250 @ 80), (N/A, 17)
    (5250 - 5330 @ 80), (N/A, 23), DFS
    (5490 - 5710 @ 80), (N/A, 23), DFS
    (5735 - 5815 @ 80), (N/A, 23)"""
    zone = Zone("WORLD10",
                ['KE', 'WS', 'SA', 'UG'],
                db_string)
    zone_list.append(zone)
	
	#WORLDWORLD
    db_string = """\
    (2402 - 2472 @ 40), (N/A, 23)
    (5170 - 5250 @ 80), (N/A, 17), DFS
    (5250 - 5330 @ 80), (N/A, 23), DFS
    (5490 - 5710 @ 80), (N/A, 23), DFS
    (5735 - 5835 @ 80), (N/A, 23), DFS"""
    zone = Zone("WORLDWORLD",
                ['DC'],
                db_string)
    zone_list.append(zone)

    return zone_list

if __name__ == "__main__":
    zone_list = build_db()

    for n in zone_list:
        print(n.name)
        print(n.db_string)
        countries = ",".join(n.country_list)
        print(countries)
        print("\n")

    f = open("db.txt", "w")

    f.write("""\
# This is the world regulatory domain
country 00:
    (2402 - 2472 @ 40), (3, 20)
    # Channel 12 - 13.
    (2457 - 2482 @ 40), (3, 20), PASSIVE-SCAN, NO-IBSS
    # Channel 14. Only JP enables this and for 802.11b only
    (2474 - 2494 @ 20), (3, 20), PASSIVE-SCAN, NO-IBSS, NO-OFDM
    # Channel 36 - 48
    (5170 - 5250 @ 80), (3, 20), PASSIVE-SCAN, NO-IBSS
    # NB: 5260 MHz - 5700 MHz requies DFS
    # Channel 149 - 165
    (5735 - 5835 @ 80), (3, 20), PASSIVE-SCAN, NO-IBSS
    # IEEE 802.11ad (60GHz), channels 1..3
    (57240 - 63720 @ 2160), (N/A, 0)

""")
    for zone in zone_list:
        zone.write_file(f)

    f.close()
