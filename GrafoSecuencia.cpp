#include "GrafoSecuencia.h"
#include <queue>
#include <limits>
#include <cmath>
#include <algorithm>

GrafoSecuencia::GrafoSecuencia(const std::string& data, std::size_t anchoLinea) {
    construirMatriz(data, anchoLinea);
    construirGrafo();
}

void GrafoSecuencia::construirMatriz(const std::string& data, std::size_t anchoLinea) {
    matriz_.clear();
    if (anchoLinea == 0) return;

    std::size_t pos = 0;
    const std::size_t n = data.size();

    while (pos < n) {
        std::size_t len = std::min(anchoLinea, n - pos);
        std::vector<char> fila;
        fila.reserve(len);
        for (std::size_t k = 0; k < len; ++k) {
            fila.push_back(data[pos + k]);
        }
        matriz_.push_back(std::move(fila));
        pos += len;
    }
}

bool GrafoSecuencia::puntoValido(const Punto2D& p) const {
    if (p.fila < 0 || p.fila >= static_cast<int>(matriz_.size())) return false;
    if (p.col  < 0 || p.col  >= static_cast<int>(matriz_[p.fila].size())) return false;
    return true;
}

bool GrafoSecuencia::existePunto(const Punto2D& p) const {
    return puntoValido(p);
}

char GrafoSecuencia::baseEn(const Punto2D& p) const {
    if (!puntoValido(p)) return '\0';
    return matriz_[p.fila][p.col];
}

double GrafoSecuencia::pesoEntre(const Punto2D& a, const Punto2D& b) const {
    char ca = baseEn(a);
    char cb = baseEn(b);
    int asciiA = static_cast<unsigned char>(ca);
    int asciiB = static_cast<unsigned char>(cb);
    return 1.0 / (1.0 + std::abs(asciiA - asciiB));
}

void GrafoSecuencia::construirGrafo() {
    ady_.clear();
    if (matriz_.empty()) return;

    const int filas = static_cast<int>(matriz_.size());

    for (int i = 0; i < filas; ++i) {
        const int cols = static_cast<int>(matriz_[i].size());
        for (int j = 0; j < cols; ++j) {
            Punto2D p{i, j};

            //arriba, abajo, izquierda, derecha
            const int di[4] = {-1, 1, 0, 0};
            const int dj[4] = {0, 0, -1, 1};

            for (int k = 0; k < 4; ++k) {
                Punto2D q{i + di[k], j + dj[k]};
                if (puntoValido(q)) {
                    double w = pesoEntre(p, q);
                    ady_[p].push_back({q, w});
                }
            }

            if (ady_.find(p) == ady_.end()) {
                ady_[p] = {};
            }
        }
    }
}

void GrafoSecuencia::dijkstraDesde(
    const Punto2D& origen,
    std::map<Punto2D, double>& dist,
    std::map<Punto2D, Punto2D>& prev
) const {
    dist.clear();
    prev.clear();

    const double INF = std::numeric_limits<double>::infinity();

    for (const auto& par : ady_) {
        dist[par.first] = INF;
    }

    if (dist.find(origen) == dist.end()) {
        return;
    }

    dist[origen] = 0.0;

    using Estado = std::pair<double, Punto2D>;
    struct Cmp {
        bool operator()(const Estado& a, const Estado& b) const {
            return a.first > b.first;
        }
    };

    std::priority_queue<Estado, std::vector<Estado>, Cmp> pq;
    pq.push({0.0, origen});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > dist[u]) continue; // entrada vieja

        auto it = ady_.find(u);
        if (it == ady_.end()) continue;

        for (const auto& vecino : it->second) {
            const Punto2D& v = vecino.first;
            double w = vecino.second;
            double nd = d + w;

            if (nd < dist[v]) {
                dist[v] = nd;
                prev[v] = u;
                pq.push({nd, v});
            }
        }
    }
}

std::vector<Punto2D> GrafoSecuencia::rutaMasCorta(
    const Punto2D& origen,
    const Punto2D& destino,
    double& costoTotal
) const {
    std::vector<Punto2D> ruta;
    costoTotal = std::numeric_limits<double>::infinity();

    if (!puntoValido(origen) || !puntoValido(destino)) {
        return ruta;
    }

    std::map<Punto2D, double> dist;
    std::map<Punto2D, Punto2D> prev;
    dijkstraDesde(origen, dist, prev);

    auto it = dist.find(destino);
    if (it == dist.end() || !std::isfinite(it->second)) {
        return ruta; // no hay camino
    }

    costoTotal = it->second;

    // Reconstruir el camino desde destino hasta origen
    Punto2D actual = destino;
    while (!(actual == origen)) {
        ruta.push_back(actual);
        auto itPrev = prev.find(actual);
        if (itPrev == prev.end()) {
            // Por si falta predecesor
            ruta.clear();
            costoTotal = std::numeric_limits<double>::infinity();
            return ruta;
        }
        actual = itPrev->second;
    }
    ruta.push_back(origen);

    std::reverse(ruta.begin(), ruta.end());
    return ruta;
}

std::vector<Punto2D> GrafoSecuencia::baseRemota(
    const Punto2D& origen,
    Punto2D& remoto,
    double& costoTotal
) const {
    std::vector<Punto2D> ruta;
    costoTotal = 0.0;
    remoto = origen;

    if (!puntoValido(origen)) {
        return ruta;
    }

    char baseObjetivo = baseEn(origen);

    std::map<Punto2D, double> dist;
    std::map<Punto2D, Punto2D> prev;
    dijkstraDesde(origen, dist, prev);

    double maxDist = -1.0;
    Punto2D mejor = origen;

    for (const auto& par : dist) {
        const Punto2D& p = par.first;
        double d = par.second;

        if (!std::isfinite(d)) continue;
        if (p == origen) continue;
        if (baseEn(p) != baseObjetivo) continue;

        if (d > maxDist) {
            maxDist = d;
            mejor = p;
        }
    }

    if (maxDist < 0.0) {
        costoTotal = 0.0;
        remoto = origen;
        ruta.push_back(origen);
        return ruta;
    }

    costoTotal = maxDist;
    remoto = mejor;

    // Reconstruir camino desde 'mejor' hasta 'origen'
    Punto2D actual = mejor;
    while (!(actual == origen)) {
        ruta.push_back(actual);
        auto itPrev = prev.find(actual);
        if (itPrev == prev.end()) {
            ruta.clear();
            costoTotal = std::numeric_limits<double>::infinity();
            return ruta;
        }
        actual = itPrev->second;
    }
    ruta.push_back(origen);
    std::reverse(ruta.begin(), ruta.end());
    return ruta;
}
