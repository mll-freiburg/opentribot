#ifndef _ColorClassifier_h_
#define _ColorClassifier_h_

#include <string>
#include "../Formation/ColorTuples.h"

namespace Tribots {

  /**
   * Ein ColorClassifier klassifiziert �bergebene Farbwerte. Die 
   * zur�ckgegebenen Werte sollten den in der verwendeten ColorClassList
   * enthaltenen Informationen entsprechen.
   *
   * \todo Eventuell Zusammenhang zwischen ColorClassList und
   *       ColorClassifier herstellen (zum Beispiel ColorClassList mit
   *       in der Lookuptabelle abspeichern). Derzeit etwas unsch�n...
   */
  class ColorClassifier {
  public:
    virtual ~ColorClassifier() {;}
    /**
     * klassifiziere das gegebene RGBTuple
     */
    virtual const unsigned char& lookup(const RGBTuple&  rgb)  const =0;
    /**
     * klassifiziere das gegebene YUVTuple
     */
    virtual const unsigned char& lookup(const YUVTuple&  yuv)  const =0;
    /**
     * klassifiziere das gegebene UYVYTuple
     */
    virtual const unsigned char& lookup(const UYVYTuple& uyvy, 
					int pos=0) const =0;

    /**
     * �ndere das Klassenlabel des �bergebenen RGBTuple rgb auf Klasse c.
     * Wie die unterschiedlichen Implementierungen mit dieser Information
     * verfahren, ist nicht n�her definiert. Eventuell k�nnen angrenzende
     * Farben ebenfalls ge�ndert oder der Aufruf g�nzlich ignoriert werden.
     *
     * \param rgb Farbwert, dessen Klassifizierung ge�ndert werden soll
     * \param c   neues Klassenlabel
     */
    virtual void set(const RGBTuple&  rgb,  unsigned char c) =0;
    /**
     * �ndere das Klassenlabel des �bergebenen YUVTuple yuv auf Klasse c.
     * Wie die unterschiedlichen Implementierungen mit dieser Information
     * verfahren, ist nicht n�her definiert. Eventuell k�nnen angrenzende
     * Farben ebenfalls ge�ndert oder der Aufruf g�nzlich ignoriert werden.
     *
     * \param yuv Farbwert, dessen Klassifizierung ge�ndert werden soll
     * \param c   neues Klassenlabel
     */
    virtual void set(const YUVTuple&  yuv,  unsigned char c) =0;
    /**
     * �ndere das Klassenlabel des �bergebenen UYVYTuple uyvy auf Klasse c.
     * Wie die unterschiedlichen Implementierungen mit dieser Information
     * verfahren, ist nicht n�her definiert. Eventuell k�nnen angrenzende
     * Farben ebenfalls ge�ndert oder der Aufruf g�nzlich ignoriert werden.
     *
     * \param uyvy Farbwert, dessen Klassifizierung ge�ndert werden soll
     * \param c    neues Klassenlabel
     */
    virtual void set(const UYVYTuple& uyvy, unsigned char c, int pos=0) =0;

    /**
     * l�dt einen Klassifikator aus einer Datei. Das Dateiformat ist vom
     * Klassifikator abh�ngig (und dort beschrieben).
     */
    virtual void load(std::string filename) =0;
    /**
     * schreibt einen Klassifikator in eine Datei. Das Dateiformat ist vom
     * Klassifikator abh�ngig (und dort beschrieben).
     */
    virtual void save(std::string filename) const =0;

    /**
     * erzeugt einen neuen Klassifikator (Kopie).
     */
    virtual ColorClassifier* create() const =0;
    /**
     * erzeugt einen neuen Klassifikator, der aus einer Datei eingelesen 
     * wird.
     */
    virtual ColorClassifier* createFromFile(std::string filename) const;
  };

  /**
   * Standardimplementierung, die f�r jeden Farbwert 0 (COLOR_IGNORE)
   * zur�ckliefert.
   */
  class DefaultClassifier : public ColorClassifier {
  public:
    virtual const unsigned char& lookup(const RGBTuple&  rgb)  const;
    virtual const unsigned char& lookup(const YUVTuple&  yuv)  const;
    virtual const unsigned char& lookup(const UYVYTuple& uyvy, 
					int pos=0) const;

    virtual void set(const RGBTuple&  rgb,  unsigned char c);
    virtual void set(const YUVTuple&  yuv,  unsigned char c);
    virtual void set(const UYVYTuple& uyvy, unsigned char c, int pos=0);

    /**
     * hat keinen Effekt
     */
    virtual void load(std::string filename);
    /**
     * hat keinen Effekt
     */
    virtual void save(std::string filename) const;

    /**
     * es ist verboten, vom DefaultClassifier eine Kopie anzulegen.
     * create() liefert daher im Falle eines Aufrufs eine Exception.
     */
    virtual ColorClassifier* create() const;

    /**
     * Liefert einen Zeiger auf die einzige Instanz des DefaultClassifier.
     */
    static const DefaultClassifier* getInstance();

  protected:
    /**
     * Da es sich um ein Singleton handelt, soll der Konstrukter nicht 
     * aufgerufen werden.
     */
    DefaultClassifier();               
    static DefaultClassifier* singleton; ///< einzige Instanz
    unsigned char defValue;              ///< default value, 0 f�r COLOR_IGNORE
  };

  /**
   * Klassifiziert schwarze Pixel (0,0,0) als Klasse 0, weisse Pixel 
   * (255, 255, 255) als Klasse 1. F�r andere Farbwerte ist die Klassifizierung
   * undefiniert.
   */
  class WhiteClassifier : public DefaultClassifier {
  public:
    WhiteClassifier();

    virtual const unsigned char& lookup(const RGBTuple&  rgb)  const;
    virtual const unsigned char& lookup(const YUVTuple&  yuv)  const;
    virtual const unsigned char& lookup(const UYVYTuple& uyvy, 
					int pos=0) const;

  protected:
    unsigned char white;
    unsigned char black;
  };

  /**
   * Klassifiziert schwarze Pixel (0,0,0) als Klasse 1, weisse Pixel 
   * (255, 255, 255) als Klasse 0. F�r andere Farbwerte ist die Klassifizierung
   * undefiniert.
   */
  class BlackClassifier : public WhiteClassifier {
  public:
    BlackClassifier();
  };

}

#endif
