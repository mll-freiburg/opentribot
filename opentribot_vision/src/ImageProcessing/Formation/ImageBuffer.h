#ifndef _imagebuffer_h_
#define _imagebuffer_h_

#include "../../Structures/TribotsException.h"
#include "../../Fundamental/Time.h"

namespace Tribots {

  /**
   * "Dünne" Schicht, die um die Bildpuffer, die von den Bildquellen geliefert
   * werden, gewickelt werden kann. Die Puffer werden so mit einem Tag 
   * versehen, das angibt, in welchem Format die Daten in dem Puffer 
   * organisiert sind. Außerdem sind Informationen über die Größe des Puffers
   * und die Zeilen und Spaltenanzahl des Bildes enthalten.
   *
   * Diese Klasse ist die gemeinsame Schnittstelle zwischen den Bildquellen 
   * und den verschiedenen Bildklassen. Sofern der Benutzer die Fassade
   * ImageProducer verwendet, kommt er mit dieser Klasse nicht in Berührung.
   *
   * \attention Diese Klasse sollte nicht direkt vom Benutzer verwendet werden.
   *            Bilder erhält man von der Klasse ImageProducer, die Bilder vom
   *            Typ Image zurückgibt.
   */
  class ImageBuffer 
  {
  public: 
    enum {
      FORMAT_YUV444=0,
      FORMAT_YUV422,
      FORMAT_YUV411,
      FORMAT_RGB,
      FORMAT_MONO,
      FORMAT_MONO16,
      FORMAT_UYV
    };      

    int width;              ///< Breite des Bildes
    int height;             ///< Höhe des Bildes
    int format;             ///< Format des Bildes (FORMAT_YUV444,...)
    unsigned char* buffer;  ///< Zeiger auf den Speicherbereich
    int size;               ///< Größe des Speicherbereichs in byte
    Time timestamp;   ///< Erzeugungszeitpunkt des Bildes

    /**
     * Erzeugt eine neue Instanz von ImageBuffer um den übergebenen Puffer
     * buffer. Der Puffer muss bereits alloziert und initialisiert worden sein.
     * Eine Plausibiliätsprüfung der Parameter width, height, format und size
     * geschieht nicht.
     *
     * \param width Breite des Bildes (in Pixeln)
     * \param height Höhe des Bildes (in Pixeln)
     * \param format Datenformat des Puffers (FORMAT_YUV444=0, FORMAT_YUV422,
     *        FORMAT_YUV411, FORMAT_RGB, FORMAT_MONO, FORMAT_MONO16, 
     *        FORMAT_UYV)
     * \param buffer Zeiger auf den Speicherbereich, der das Bild enthält
     * \param size Größe des Speicherbereichs in byte
     */
    ImageBuffer(int width=0, int height=0, int format=0, 
		unsigned char* buffer=0, int size=0);
    ImageBuffer(int width, int height, int format, 
                unsigned char* buffer, int size, Time ts);

    /**
     * Konvertiert den Inhalt eines Puffers in das Datenformat eines anderen
     * Puffers. Der Benutzer hat selbst dafür zu sorgen, dass die Puffergrößen
     * der übergebenen ImageBuffer zueinander passen.
     *
     * Diese Methode sucht eine passende Konvertierungsfunktion aus einer
     * zentralen Registrierung. Ist eine benötigte Konvertierung nicht 
     * implementiert, schlägt der Aufruf mit einer Exception fehl.
     *
     * \param src Der Puffer, von dem kopiert werden soll.
     * \param dst Der Puffer, in den geschrieben werden soll. Das Datenformat
     *            des Puffers wird beachtet, die Größe muss zu src passen.
     */
    static void convert(const ImageBuffer& src, ImageBuffer& dst) 
      throw (TribotsException);			
  };

}

#endif
