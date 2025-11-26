#ifndef FASTA_H
#define FASTA_H

#include <string>
#include <vector>
#include "Secuencia.h"
#include <unordered_map>
#include <cstdint>
#include "Huffarbol.h"

class Fasta {
private:
    std::vector<Secuencia> secuencias_; // Estructura lineal para almacenar las secuencias
    std::vector<size_t> lineWidths_; 

public:
    
    size_t cargar(const std::string& nombre_archivo); //funcion para cargar un archivo 
    bool guardar(const std::string& nombre_archivo) const;//funcion para guardar en un archivo las secuencisa con la respetiva subseq enmmascarada
    const std::vector<Secuencia>& secuencias() const { return secuencias_; } //funcion para listar
    size_t contarSubsecuencia(const std::string& subseq) const; //funcion para hallar si una subseq dada por el usuario existe 
    std::unordered_map<char, size_t> obtenerHistograma(const std::string& descripcion) const;//funcion para generar el histograma
    size_t enmascararSubsecuencia(const std::string& subseq);//funcion para enmascarar una subseq dada por el usuario
    bool codificarHuffman(const std::string& nombre_archivo) const;
    bool decodificarHuffman(const std::string& nombre_archivo); // ← NUEVA
    bool haySecuencias() const { return !secuencias_.empty(); }  // función para verificar si hay secuencias cargadas
    int buscarIndicePorDescripcion(const std::string& descripcion) const;  // función para obtener el índice de una secuencia por su descripción
    std::size_t getLineWidth(std::size_t idx) const;  // función para obtener el ancho de línea (columnas) de una secuencia dada
    const Secuencia& getSecuencia(std::size_t idx) const { return secuencias_.at(idx); }  // función para acceder a la secuencia en la posición idx


  
};

#endif // FASTADB_H
