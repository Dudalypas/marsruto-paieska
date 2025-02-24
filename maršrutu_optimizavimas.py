from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def sukurti_duomenu_modeli():
    """Apibrėžia atstumų matricos duomenis ir maršrutų optimizavimo parametrus."""
    duomenys = {
        "atstumu_matrica": [
            [0, 2, 9, 10],  # Sandėlis
            [1, 0, 6, 4],  # Klientas 1
            [15, 7, 0, 8],  # Klientas 2
            [6, 3, 12, 0],  # Klientas 3
        ],
        "furgonu_skaicius": 1,  # Vienas pristatymo automobilis
        "sandelio_indeksas": 0,  # Sandėlis kaip pradžios taškas
    }
    return duomenys


def optimizuoti_marsruta():
    """Sprendžia transporto maršrutų optimizavimo uždavinį."""
    duomenys = sukurti_duomenu_modeli()

    # Sukuriame maršrutų modelį
    tvarkyklė = pywrapcp.RoutingIndexManager(
        len(duomenys["atstumu_matrica"]), duomenys["furgonu_skaicius"], duomenys["sandelio_indeksas"]
    )
    marsrutu_modelis = pywrapcp.RoutingModel(tvarkyklė)

    # Apibrėžiame atstumų skaičiavimo funkciją
    def atstumo_funkcija(nuo_indekso, iki_indekso):
        """Grąžina atstumą tarp dviejų vietų."""
        nuo_tasko = tvarkyklė.IndexToNode(nuo_indekso)
        iki_tasko = tvarkyklė.IndexToNode(iki_indekso)
        return duomenys["atstumu_matrica"][nuo_tasko][iki_tasko]

    atstumu_funkcijos_indeksas = marsrutu_modelis.RegisterTransitCallback(atstumo_funkcija)

    # Nustatome, kad atstumas tarp taškų būtų optimizavimo kriterijus
    marsrutu_modelis.SetArcCostEvaluatorOfAllVehicles(atstumu_funkcijos_indeksas)

    # Nustatome paieškos strategiją
    paieskos_parametrai = pywrapcp.DefaultRoutingSearchParameters()
    paieskos_parametrai.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Sprendžiame optimizavimo uždavinį
    sprendimas = marsrutu_modelis.SolveWithParameters(paieskos_parametrai)

    # Išvedame rezultatą
    if sprendimas:
        spausdinti_sprendima(tvarkyklė, marsrutu_modelis, sprendimas)
    else:
        print("Sprendimo nepavyko rasti.")


def spausdinti_sprendima(tvarkyklė, marsrutu_modelis, sprendimas):
    """Atspausdina optimizuotą maršrutą."""
    print("Optimalus maršrutas:")
    indeksas = marsrutu_modelis.Start(0)
    marsrutas = []
    while not marsrutu_modelis.IsEnd(indeksas):
        marsrutas.append(tvarkyklė.IndexToNode(indeksas))
        indeksas = sprendimas.Value(marsrutu_modelis.NextVar(indeksas))
    marsrutas.append(tvarkyklė.IndexToNode(indeksas))
    print(" -> ".join(map(str, marsrutas)))


# Vykdome optimizavimo algoritmą
optimizuoti_marsruta()
