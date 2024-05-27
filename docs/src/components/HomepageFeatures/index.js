import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    img_name: 'Caio Teixeira de Paula',
    img_src: "https://media.licdn.com/dms/image/D4D03AQGBJLYhBOwqUQ/profile-displayphoto-shrink_200_200/0/1680548400615?e=1719446400&v=beta&t=NDsYkD3dlB9r7Kal7ikkPdMCKIRwaXaw7wzoP2BqNe0",
    linkedin : "https://www.linkedin.com/in/caio-teixeira-paula/",
  },
  {
    img_name: 'Cecília Alonço Gonçalves',
    img_src: "https://media.licdn.com/dms/image/D4E03AQHFDADl2nqTcA/profile-displayphoto-shrink_200_200/0/1680660675815?e=1719446400&v=beta&t=ucOjGB_bF8KICbk3Qcm0G8zwiO7ZMjxnnojKFYjQ30k",
    linkedin: "https://www.linkedin.com/in/cec%C3%ADlia-alonso-gon%C3%A7alves-3aa4bb271/",
  },
  {
    img_name: 'Eduardo do Santos',
    img_src: "https://media.licdn.com/dms/image/D4D03AQEJckPVxSd3HA/profile-displayphoto-shrink_200_200/0/1686266549436?e=1719446400&v=beta&t=RxXGe7VWGDOXYaALPzZggVoTtTuhBsPsaiPQ0hOQowM",
    linkedin: "https://www.linkedin.com/in/eduardo-henrique-dos-santos/",
  },
  {
    img_name: 'José Vitor Alencar',
    img_src: "https://media.licdn.com/dms/image/D4D03AQGcVfLbFU12Ww/profile-displayphoto-shrink_200_200/0/1714085847839?e=1720051200&v=beta&t=6y9gs-vKsRwPS5WlDT5DD-ApX-rTWJouktmyb81l2a0",
    linkedin: "https://www.linkedin.com/in/josevalencar/",
  },
  {
    img_name: "Lídia Cruz Mariano",
    img_src : "https://media.licdn.com/dms/image/D4D03AQG56mwRJ4G55g/profile-displayphoto-shrink_200_200/0/1675023865459?e=1719446400&v=beta&t=RGMxoQ_mjRd1T5Azg3dt-T1RgxicmchyGsP3qGYw4OM",
    linkedin: "https://www.linkedin.com/in/lidiamariano/",
  },
  {
    img_name : "Murilo Prianti",
    img_src : "https://media.licdn.com/dms/image/D4D35AQG6W_7TsJCfoQ/profile-framedphoto-shrink_200_200/0/1655926445979?e=1714446000&v=beta&t=zoyHGNmwU6g3QYrzVUsT_XzIjUleoT5sz-AqLf_s254",
    linkedin : "https://www.linkedin.com/in/murilo-prianti-0073111a1/",
  },
  {
    img_name : "Pedro Coutinho Cruz",
    img_src : "https://media.licdn.com/dms/image/D4E35AQHjfLOaQ5ZbqQ/profile-framedphoto-shrink_800_800/0/1695676563992?e=1714586400&v=beta&t=eGIJE2g92JtoiOVLJHwqi1SIGg_MIopMdqwYfq1RwyM",
    linkedin : "https://www.linkedin.com/in/pedro-henrique-coutinho-cruz/",
  }
];

function Feature({ img_src, img_name, linkedin}) {
  return (
    <div className={clsx('col col--3', styles.member_card)}>
      <a
        href={linkedin}
        target='_blank'
        rel='noopener noreferrer'
      >
        <div className="text--center">
          <img src={img_src} className={styles.roundedImage} alt={img_name} />
        </div>
        <div className="text--center padding-horiz--md">
          <Heading as="h3">{img_name}</Heading>
        </div>
      </a>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container text--center gap-2">
        <Heading as="h1">Desenvolvedores</Heading>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}