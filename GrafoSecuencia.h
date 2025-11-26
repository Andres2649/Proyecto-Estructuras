#ifndef GRAFOSECUENCIA_H
#define GRAFOSECUENCIA_H

#include <vector>
#include <map>
#include <string>
#include "Punto2D.h"

// TAD GrafoSecuencia
// Representa el grafo construido a partir de la secuencia FASTA:
// - La secuencia se ve como una matriz (filas x columnas).
// - Cada celda [i,j] es un vértice del TAD Punto2D.

class GrafoSecuencia {
public:
    // Construye la matriz
    GrafoSecuencia(const std::string& data, std::size_t anchoLinea);

    // Existe
    bool existePunto(const Punto2D& p) const;

    // Devuelve la base (caracter) en la posición dada.
    // Si el punto no es válido, devuelve '\0'.
    char baseEn(const Punto2D& p) const;

    // Calcula la ruta más corta (camino de menor costo) entre origen y destino
    // usando el grafo construido. Devuelve el vector de puntos en la ruta
    // (incluye origen y destino) y deja el costo total en costoTotal.
    // Si no hay camino, la ruta devuelta estará vacía.
    std::vector<Punto2D> rutaMasCorta(
        const Punto2D& origen,
        const Punto2D& destino,
        double& costoTotal
    ) const;

    // Calcula la "base remota": la misma letra que la base en 'origen'
    // que esté más lejos (mayor costo de ruta) dentro de la matriz.
    // Devuelve la ruta desde origen hasta el punto remoto, y deja:
    // - remoto: coordenada de la base remota
    // - costoTotal: costo total de la ruta
    // Si no existe otra base igual, la ruta será solo el origen y costo 0.
    std::vector<Punto2D> baseRemota(
        const Punto2D& origen,
        Punto2D& remoto,
        double& costoTotal
    ) const;

private:
    // Matriz de caracteres que representa la secuencia.
    // matriz_[i][j] = base en la fila i, columna j.
    std::vector<std::vector<char>> matriz_;

    // Lista de adyacencia del grafo:
    // Para cada Punto2D, se guarda un vector de (vecino, peso).
    std::map<Punto2D, std::vector<std::pair<Punto2D, double>>> ady_;

    // Construye la matriz a partir del string lineal y el ancho de línea.
    void construirMatriz(const std::string& data, std::size_t anchoLinea);

    // Construye la lista de adyacencia 'ady_' conectando cada celda con
    // sus vecinos válidos (arriba, abajo, izquierda, derecha).
    void construirGrafo();

    // Verifica si el punto está dentro de los límites de la matriz.
    bool puntoValido(const Punto2D& p) const;

    // Calcula el peso de la arista entre dos puntos válidos,
    // usando la fórmula:
    //   1 / (1 + |ASCII(a) - ASCII(b)|)
    double pesoEntre(const Punto2D& a, const Punto2D& b) const;

    // Implementa Dijkstra desde 'origen', dejando los resultados en:
    // - dist[p]: costo mínimo desde origen hasta p
    // - prev[p]: predecesor de p en el camino mínimo
    void dijkstraDesde(
        const Punto2D& origen,
        std::map<Punto2D, double>& dist,
        std::map<Punto2D, Punto2D>& prev
    ) const;
};

#endif // GRAFOSECUENCIA_H
